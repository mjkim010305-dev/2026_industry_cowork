#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Green box detector node.

No object-recognition AI (Session 11 scope): the box is found by HSV colour
thresholding on the RGB stream, and its distance/centre is resolved by fusing
the blob's pixel column with the nearest /scan return that projects inside
that column. All geometry/vision math lives in ``geometry.py`` (ROS-free,
unit tested); this node is only the ROS plumbing around it - ONE class of io
wiring - subscribe, convert, call geometry, transform, publish - the same
perception/action split ``vlm_movability_node.py`` documents for this
codebase (Python nodes publish, BT actions act).

Session 11 R2 rewrite: the association between the blob and /scan is done
entirely in 3D with the FULL camera<->scan TF (rotation + translation), never
a yaw-only approximation - see ``geometry.py`` module docstring for why that
mattered (optical-frame mounts, parallax at close range). ``camera_frame``
must therefore be an OPTICAL frame (REP-103: x right, y down, z forward);
this is what the pinhole projection in ``geometry.select_points_in_column_window``
assumes.

Two ranging paths, per the Session 11 contract:
  * lidar path (primary) - scan returns are converted to 3D points and
    transformed into ``camera_frame``, then ``geometry.select_points_in_
    column_window`` keeps the ones in front of the camera that project into
    the blob's pixel columns (+/- ``bearing_margin_px``); ``geometry.
    contiguous_cluster_centroid`` takes the contiguous surface around the
    closest of those (``cluster_gap_m``) and uses its centroid as the box's
    front face.
  * camera-fallback path - used only when the lidar path finds nothing in
    view (lidar plane above/below the box - see README limits). Depth comes
    from the known box height and the blob's pixel height via
    ``geometry.fallback_point_from_height``.
Which path fired is written into the debug image text and logged at debug
level, per the contract (source must be visible, not just inferred).

Session 11 R4: two guards against a partially occluded blob being resolved
against the wrong object (g6 live-run defect - see geometry.py module
docstring). (1) a blob narrower than ``min_width_px`` is rejected outright,
this frame counting as "no detection". (2) even a wide-enough blob's resolved
position is only published as a detection once the last ``confirm_frames``
consecutive frames were all accepted and agree within ``confirm_radius`` of
their mean (``geometry.confirm_window_mean``); any rejected or unresolved
frame clears the confirmation window. ``green_box/detected`` still publishes
every image, as before - it is just ``False`` more often now.
"""

from collections import deque
import threading

import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_system_default
from rclpy.time import Time

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

# festa_demo: script-dir import (geometry.py installed alongside this file
# in lib/festa_demo), was `from green_box_approach.geometry import ...`
from geometry import (
    bbox_touches_vertical_border,
    box_center_from_front,
    confirm_window_mean,
    contiguous_cluster_centroid,
    fallback_point_from_height,
    hsv_mask_to_blob,
    is_bbox_wide_enough,
    scan_points_xyz,
    select_points_in_column_window,
    transform_point,
    transform_points,
)


class GreenBoxDetector(Node):

    def __init__(self):
        super().__init__("green_box_detector")

        # --- topics (Session 11 contract - real TurtleBot3 names unknown,
        # every one is a parameter so deployment only needs a launch/yaml
        # override, never a code change) --------------------------------
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/image_raw").value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/camera_info").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.output_frame = self.declare_parameter("output_frame", "map").value
        # "" -> use the image message's own header.frame_id. Whichever frame
        # is used MUST be an OPTICAL frame per REP-103 (x right, y down, z
        # forward) - the pinhole projection below assumes that convention.
        # In this sim the image header frame is a non-optical identity
        # frame, so a real run overrides this with e.g. "camera_optical".
        self.camera_frame = self.declare_parameter("camera_frame", "").value

        # --- HSV thresholding ---------------------------------------------
        # Placeholder green range (pure green in OpenCV HSV, H 0-179). Real
        # box colour/lighting is unknown before the demo course is built -
        # PLACEHOLDER, nav team must retune on the actual box/lighting
        # (see README "HSV 튜닝 절차").
        self.hsv_lower = tuple(self.declare_parameter(
            "hsv_lower", [40, 60, 40]).value)
        self.hsv_upper = tuple(self.declare_parameter(
            "hsv_upper", [80, 255, 255]).value)
        self.min_area = int(self.declare_parameter("min_area", 200).value)
        self.morph_kernel = int(self.declare_parameter("morph_kernel", 5).value)

        # --- Session 11 R4 guards (g6 live-run: an occluded sliver got
        # resolved against the wall behind the box) - see geometry.py
        # module docstring for the reasoning behind both defaults.
        self.min_width_px = int(self.declare_parameter("min_width_px", 20).value)
        # Session 11 R7b: at contact range the box overflows the frame
        # top/bottom, clipping the blob's pixel height that the
        # camera-fallback path's similar-triangles depth relies on - see
        # geometry.bbox_touches_vertical_border docstring.
        self.edge_margin_px = int(self.declare_parameter("edge_margin_px", 3).value)
        self.confirm_frames = int(self.declare_parameter("confirm_frames", 3).value)
        self.confirm_radius = float(self.declare_parameter("confirm_radius", 0.15).value)
        self._confirm_window = deque(maxlen=self.confirm_frames)

        # --- lidar fusion ---------------------------------------------------
        # Pixel margin (not radians, see Session 11 R2) widening the blob's
        # bbox columns before matching projected scan points against them.
        self.bearing_margin_px = self.declare_parameter("bearing_margin_px", 20.0).value
        # Max spacing [m] between consecutive scan points of one surface -
        # see geometry.contiguous_cluster_centroid.
        self.cluster_gap_m = self.declare_parameter("cluster_gap_m", 0.05).value
        self.range_min = self.declare_parameter("range_min", 0.05).value
        self.range_max = self.declare_parameter("range_max", 8.0).value

        # --- box dimensions - PLACEHOLDER, nav team must measure the real
        # demo box and overwrite both (see README "알려진 한계"). Used only
        # by the camera-fallback path (box_height_m) and to locate the box
        # centre from its detected front face (box_depth_m).
        self.box_height_m = self.declare_parameter("box_height_m", 0.30).value
        self.box_depth_m = self.declare_parameter("box_depth_m", 0.30).value

        self.tf_timeout = self.declare_parameter("tf_timeout", 0.2).value
        # Stamp-exact fusion (L_corridor position tests: while the robot
        # turned, the old code projected the image with a TF seconds out of
        # date and the box estimate swung around the robot by up to ~0.4 m).
        # Scan points are moved into `fixed_frame` at the SCAN stamp and back
        # into the camera at the IMAGE stamp, so robot motion between the two
        # captures cancels. `fixed_frame` must be smooth over a second or so
        # (odom, not map). A scan further than `max_scan_image_dt` from the
        # image, or a missing TF at either stamp, drops the frame - there is
        # no fallback to the latest TF any more.
        self.fixed_frame = self.declare_parameter("fixed_frame", "odom").value
        self.max_scan_image_dt = self.declare_parameter("max_scan_image_dt", 0.1).value

        self.bridge = CvBridge()
        self.intrinsics = None          # (fx, fy, cx, cy)
        self._scans = deque(maxlen=20)  # recent sensor_msgs/LaserScan, oldest first
        self._scans_lock = threading.Lock()

        # TF callbacks live in the listener's reentrant group; with the
        # MultiThreadedExecutor in main() they keep filling the buffer while
        # an image callback waits in lookup_transform (the single-threaded
        # executor used to starve them, so every stamped lookup timed out).
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        image_group = MutuallyExclusiveCallbackGroup()
        scan_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, self.image_topic, self._on_image, sensor_qos, callback_group=image_group)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos,
            callback_group=scan_group)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, sensor_qos, callback_group=scan_group)

        self.pose_pub = self.create_publisher(
            PoseStamped, "green_box/pose", qos_profile_system_default)
        self.detected_pub = self.create_publisher(
            Bool, "green_box/detected", qos_profile_system_default)
        self.debug_image_pub = self.create_publisher(Image, "green_box/debug_image", sensor_qos)

        self.get_logger().info(
            "green_box_detector: image='%s' scan='%s' -> green_box/pose,green_box/detected "
            "(output_frame='%s')" % (self.image_topic, self.scan_topic, self.output_frame))

    # ------------------------------------------------------------------
    # subscriptions

    def _on_camera_info(self, msg):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = (k[0], k[4], k[2], k[5])
            self.get_logger().info(
                "green_box_detector: intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f" %
                self.intrinsics)

    def _on_scan(self, msg):
        with self._scans_lock:
            self._scans.append(msg)

    def _scan_nearest_to(self, stamp):
        """Buffered scan whose stamp is closest to ``stamp`` and the gap in
        seconds, or ``(None, None)`` if no scan has arrived yet."""
        t = Time.from_msg(stamp).nanoseconds
        with self._scans_lock:
            if not self._scans:
                return None, None
            scan = min(self._scans, key=lambda s: abs(Time.from_msg(s.header.stamp).nanoseconds - t))
        return scan, abs(Time.from_msg(scan.header.stamp).nanoseconds - t) * 1e-9

    def _on_image(self, msg):
        if self.intrinsics is None:
            return
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001 - log and drop
            self.get_logger().warn("green_box_detector: image convert failed: %s" % exc)
            return

        fx, fy, cx, cy = self.intrinsics
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.hsv_lower), np.array(self.hsv_upper))
        if self.morph_kernel > 1:
            kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        bbox = hsv_mask_to_blob(mask, self.min_area)
        detected = False
        source = "none"
        pose_xy = None
        reason = "no-blob"

        if bbox is None:
            self._confirm_window.clear()
        elif not is_bbox_wide_enough(bbox, self.min_width_px):
            # g6: an occluded sliver still resolves to *a* range, just the
            # wrong object's - reject before ranging at all, and this frame
            # is a rejection, not a detection, for the confirmation window.
            reason = "narrow"
            self._confirm_window.clear()
        else:
            pose_xy, source = self._resolve_front(
                msg.header, bbox, fx, fy, cx, cy, bgr.shape[0])
            if pose_xy is None:
                reason = source if source in ("clipped", "unsynced", "no-tf") else "no-range"
                # A frame dropped for missing data (no scan close enough in
                # time, no TF at the stamp) says nothing about the box, so it
                # must not reset the confirmation run the way a rejected
                # measurement does - otherwise every drop opens a gap in
                # green_box/pose and FinalApproachStop's pose wait times out.
                if source not in ("unsynced", "no-tf"):
                    self._confirm_window.clear()
            else:
                self._confirm_window.append(pose_xy)
                if len(self._confirm_window) < self.confirm_frames:
                    reason = "confirming %d/%d" % (len(self._confirm_window), self.confirm_frames)
                    pose_xy = None
                else:
                    mean_xy = confirm_window_mean(list(self._confirm_window), self.confirm_radius)
                    if mean_xy is None:
                        # Frames disagree: start over, as the docstring and
                        # README promise for every rejected frame.
                        self._confirm_window.clear()
                        reason = "unstable"
                        pose_xy = None
                    else:
                        reason = "confirmed"
                        pose_xy = mean_xy
                        detected = True

        if detected:
            self._publish_pose(msg.header, pose_xy)
        # Same reasoning as the confirmation window above: a dropped frame is
        # "no measurement", not "no box", so green_box/detected keeps its
        # last value instead of flickering to False.
        if reason not in ("unsynced", "no-tf"):
            self.detected_pub.publish(Bool(data=detected))
        self._publish_debug_image(bgr, mask, bbox, detected, source, reason)

    # ------------------------------------------------------------------
    # ranging

    def _resolve_front(self, image_header, bbox, fx, fy, cx, cy, image_height):
        """Box centre ``(x, y)`` in ``output_frame``, or ``None`` if neither
        the lidar nor the camera-fallback path could resolve one. Also
        returns which path fired (``"lidar" | "camera-fallback" | "clipped" |
        "none"``).

        ``camera_frame`` (param, "" = ``image_header.frame_id``) MUST be an
        OPTICAL frame - see module docstring and README.
        """
        x, y, w, h = bbox
        camera_frame = self.camera_frame or image_header.frame_id
        u_lo = x - self.bearing_margin_px
        u_hi = x + w + self.bearing_margin_px

        front_cam = None
        source = "none"
        scan, scan_dt = self._scan_nearest_to(image_header.stamp)
        if scan is not None:
            if scan_dt > self.max_scan_image_dt:
                return None, "unsynced"
            # scan -> fixed_frame at the scan stamp, fixed_frame -> camera at
            # the image stamp: robot motion between the captures cancels.
            tf_sf = self._lookup_transform(
                self.fixed_frame, scan.header.frame_id, scan.header.stamp, "scan->fixed")
            tf_fc = self._lookup_transform(
                camera_frame, self.fixed_frame, image_header.stamp, "fixed->camera")
            if tf_sf is None or tf_fc is None:
                return None, "no-tf"
            scan_pts = scan_points_xyz(
                scan.ranges, scan.angle_min, scan.angle_increment,
                self.range_min, self.range_max)
            cam_pts = transform_points(transform_points(scan_pts, *tf_sf), *tf_fc)
            kept = select_points_in_column_window(cam_pts, fx, cx, u_lo, u_hi)
            front_cam = contiguous_cluster_centroid(kept, self.cluster_gap_m)
            if front_cam is not None:
                source = "lidar"

        if front_cam is None:
            # R7b: lidar found nothing under the blob (this branch), and the
            # blob touches the image's top/bottom border - the box is closer
            # than the frame can show, so its pixel height is clipped and
            # the fallback's similar-triangles depth below would be wrong.
            # The lidar path is unaffected by this check: its range comes
            # from an actual 3D return, not the blob's height.
            if bbox_touches_vertical_border(bbox, image_height, self.edge_margin_px):
                return None, "clipped"
            # Camera-fallback: depth from known box height, back-projected
            # through the blob centre pixel (camera optical frame).
            u_c, v_c = x + w / 2.0, y + h / 2.0
            try:
                front_cam = fallback_point_from_height(
                    u_c, v_c, h, self.box_height_m, fx, fy, cx, cy)
            except ValueError:
                return None, "camera-fallback"
            source = "camera-fallback"

        tf_co = self._lookup_transform(
            self.output_frame, camera_frame, image_header.stamp, "camera->output")
        if tf_co is None:
            return None, "no-tf"
        translation, quat = tf_co
        front_out = transform_point(front_cam, translation, quat)
        camera_origin_xy = (translation[0], translation[1])
        try:
            pose_xy = box_center_from_front(
                (front_out[0], front_out[1]), camera_origin_xy, self.box_depth_m)
        except ValueError:
            return None, source
        return pose_xy, source

    def _lookup_transform(self, target_frame, source_frame, stamp, context):
        """TF lookup at exactly ``stamp`` (waiting up to ``tf_timeout``),
        returning the full ``((tx, ty, tz), (qx, qy, qz, qw))`` transform (not
        just its yaw) or ``None`` if unavailable - contract requirement: log
        throttled and skip, never guess. The old fallback to the latest TF is
        gone: while the robot turns, "latest" can be seconds away from the
        capture and was the source of the swinging box estimates.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=self.tf_timeout))
        except TransformException as exc:
            self.get_logger().warn(
                "green_box_detector: TF %s->%s unavailable (%s): %s" %
                (source_frame, target_frame, context, exc),
                throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    # ------------------------------------------------------------------
    # publishing

    def _publish_pose(self, header, xy):
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def _publish_debug_image(self, bgr, mask, bbox, detected, source, reason):
        debug = bgr.copy()
        outline = cv2.bitwise_and(bgr, bgr, mask=mask)
        debug = cv2.addWeighted(debug, 0.6, outline, 0.4, 0)
        if bbox is not None:
            x, y, w, h = bbox
            # Green + "MOVABLE" once confirmed (what the BT acts on), yellow
            # with the reason while it is still confirming or was rejected.
            colour = (0, 200, 0) if detected else (0, 200, 255)
            tag = "MOVABLE: green box" if detected else reason
            cv2.rectangle(debug, (x, y), (x + w, y + h), colour, 3)
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            ty = max(th + 8, y)
            cv2.rectangle(debug, (x, ty - th - 8), (x + tw + 10, ty), colour, -1)
            cv2.putText(debug, tag, (x + 5, ty - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (20, 20, 20), 2, cv2.LINE_AA)
        text = "detected=%s source=%s reason=%s" % (detected, source, reason)
        cv2.putText(debug, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 1, cv2.LINE_AA)
        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001 - log and drop
            self.get_logger().warn("green_box_detector: debug image convert failed: %s" % exc)
            return
        self.debug_image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GreenBoxDetector()
    # Multi-threaded so TF and scan callbacks keep running while an image
    # callback waits in a stamped lookup_transform (see fixed_frame notes).
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
