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
    nearest_by_range`` picks the closest of those as the box's front face.
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

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_system_default
from rclpy.time import Time

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from green_box_approach.geometry import (
    box_center_from_front,
    confirm_window_mean,
    fallback_point_from_height,
    hsv_mask_to_blob,
    is_bbox_wide_enough,
    nearest_by_range,
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
        self.confirm_frames = int(self.declare_parameter("confirm_frames", 3).value)
        self.confirm_radius = float(self.declare_parameter("confirm_radius", 0.15).value)
        self._confirm_window = deque(maxlen=self.confirm_frames)

        # --- lidar fusion ---------------------------------------------------
        # Pixel margin (not radians, see Session 11 R2) widening the blob's
        # bbox columns before matching projected scan points against them.
        self.bearing_margin_px = self.declare_parameter("bearing_margin_px", 20.0).value
        self.range_min = self.declare_parameter("range_min", 0.05).value
        self.range_max = self.declare_parameter("range_max", 8.0).value

        # --- box dimensions - PLACEHOLDER, nav team must measure the real
        # demo box and overwrite both (see README "알려진 한계"). Used only
        # by the camera-fallback path (box_height_m) and to locate the box
        # centre from its detected front face (box_depth_m).
        self.box_height_m = self.declare_parameter("box_height_m", 0.30).value
        self.box_depth_m = self.declare_parameter("box_depth_m", 0.30).value

        self.tf_timeout = self.declare_parameter("tf_timeout", 0.2).value

        self.bridge = CvBridge()
        self.intrinsics = None          # (fx, fy, cx, cy)
        self.last_scan = None           # sensor_msgs/LaserScan

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, sensor_qos)

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
        self.last_scan = msg

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
            pose_xy, source = self._resolve_front(msg.header, bbox, fx, fy, cx, cy)
            if pose_xy is None:
                reason = "no-range"
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
        self.detected_pub.publish(Bool(data=detected))
        self._publish_debug_image(bgr, mask, bbox, detected, source, reason)

    # ------------------------------------------------------------------
    # ranging

    def _resolve_front(self, image_header, bbox, fx, fy, cx, cy):
        """Box centre ``(x, y)`` in ``output_frame``, or ``None`` if neither
        the lidar nor the camera-fallback path could resolve one. Also
        returns which path fired (``"lidar" | "camera-fallback" | "none"``).

        ``camera_frame`` (param, "" = ``image_header.frame_id``) MUST be an
        OPTICAL frame - see module docstring and README.
        """
        x, y, w, h = bbox
        camera_frame = self.camera_frame or image_header.frame_id
        u_lo = x - self.bearing_margin_px
        u_hi = x + w + self.bearing_margin_px

        front_cam = None
        source = "none"
        scan = self.last_scan
        if scan is not None:
            tf_sc = self._lookup_transform(
                camera_frame, scan.header.frame_id, scan.header.stamp, "scan->camera")
            if tf_sc is not None:
                translation, quat = tf_sc
                scan_pts = scan_points_xyz(
                    scan.ranges, scan.angle_min, scan.angle_increment,
                    self.range_min, self.range_max)
                cam_pts = transform_points(scan_pts, translation, quat)
                kept = select_points_in_column_window(cam_pts, fx, cx, u_lo, u_hi)
                front_cam = nearest_by_range(kept)
                if front_cam is not None:
                    source = "lidar"

        if front_cam is None:
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
            return None, source
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
        """TF lookup with the existing stamp-then-latest fallback pattern,
        returning the full ``((tx, ty, tz), (qx, qy, qz, qw))`` transform (not
        just its yaw) or ``None`` if unavailable - contract requirement: log
        throttled and skip, never guess.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
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
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 0, 255), 2)
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
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
