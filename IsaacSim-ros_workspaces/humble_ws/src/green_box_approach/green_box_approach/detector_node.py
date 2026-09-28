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
the blob's horizontal bearing with the nearest /scan return in that bearing
span. All geometry/vision math lives in ``geometry.py`` (ROS-free, unit
tested); this node is only the ROS plumbing around it - ONE class of io wiring
- subscribe, convert, call geometry, transform, publish - the same
perception/action split ``vlm_movability_node.py`` documents for this
codebase (Python nodes publish, BT actions act).

Two ranging paths, per the Session 11 contract:
  * lidar path (primary) - ``geometry.select_scan_span`` picks /scan points
    whose bearing (as seen from the camera) falls inside the blob's angular
    span, ``geometry.nearest_cluster`` picks the closest of those as the
    box's front face.
  * camera-fallback path - used only when the lidar path finds nothing in
    span (lidar plane above/below the box - see README limits). Range comes
    from the known box height and the blob's pixel height via
    ``geometry.fallback_range_from_height``.
Which path fired is written into the debug image text and logged at debug
level, per the contract (source must be visible, not just inferred).
"""

import math

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
    bearing_from_pixel,
    box_center_from_front,
    fallback_range_from_height,
    hsv_mask_to_blob,
    nearest_cluster,
    select_scan_span,
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

        # --- lidar fusion ---------------------------------------------------
        self.bearing_margin = self.declare_parameter("bearing_margin", 0.05).value
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

        if bbox is not None:
            x, y, w, h = bbox
            bearing_left = bearing_from_pixel(x, fx, cx)
            bearing_right = bearing_from_pixel(x + w, fx, cx)
            bearing_lo, bearing_hi = min(bearing_left, bearing_right), max(
                bearing_left, bearing_right)

            front, ray_xy, source = self._resolve_front(
                msg, bearing_lo, bearing_hi, h, fy)
            if front is not None:
                pose_xy = box_center_from_front(front, ray_xy, self.box_depth_m)
                detected = True

        if detected:
            self._publish_pose(msg.header, pose_xy)
        self.detected_pub.publish(Bool(data=detected))
        self._publish_debug_image(bgr, mask, bbox, detected, source)

    # ------------------------------------------------------------------
    # ranging

    def _resolve_front(self, image_header, bearing_lo, bearing_hi, pixel_h, fy):
        """Front-face point (x, y) in ``output_frame`` plus the unit ray
        from sensor to box, or ``(None, None, source)`` if neither the
        lidar nor the camera-fallback path could resolve a range.
        """
        scan = self.last_scan
        if scan is not None:
            yaw = self._camera_to_scan_yaw(image_header, scan.header)
            if yaw is not None:
                # Blob bearing is measured in the camera's own horizontal
                # plane (bearing_from_pixel); rotate the window into the
                # scan frame's own angle convention before matching against
                # scan.angle_min/angle_increment, per contract ("bearing as
                # seen from the camera" mapped onto the scan via camera->
                # scan TF, not compared raw across two different frames).
                points = select_scan_span(
                    scan.ranges, scan.angle_min, scan.angle_increment,
                    bearing_lo + yaw, bearing_hi + yaw, self.bearing_margin,
                    self.range_min, self.range_max)
                nearest = nearest_cluster(points)
                if nearest is not None:
                    angle, r = nearest
                    ray_scan = (math.cos(angle), math.sin(angle))
                    front = self._to_output_frame(
                        scan.header, ray_scan[0] * r, ray_scan[1] * r)
                    ray_out = self._ray_to_output_frame(scan.header, ray_scan)
                    if front is not None and ray_out is not None:
                        return front, ray_out, "lidar"

        # Camera-fallback: range from known box height, ray straight along
        # the mean bearing of the blob's span (camera's own horizontal plane).
        try:
            r = fallback_range_from_height(pixel_h, self.box_height_m, fy)
        except ValueError:
            return None, None, "camera-fallback"
        mean_bearing = (bearing_lo + bearing_hi) / 2.0
        ray_cam = (math.cos(mean_bearing), math.sin(mean_bearing))
        front = self._to_output_frame(image_header, ray_cam[0] * r, ray_cam[1] * r)
        ray_out = self._ray_to_output_frame(image_header, ray_cam)
        if front is None or ray_out is None:
            return None, None, "camera-fallback"
        return front, ray_out, "camera-fallback"

    def _camera_to_scan_yaw(self, image_header, scan_header):
        """Yaw (rad) of the rotation that carries the camera's horizontal
        bearing plane into the scan frame's angle convention, via TF. This
        is what lets a blob bearing computed in the camera frame be matched
        against scan angles that are defined in the scan frame - without it
        the two would be compared as if the sensors shared an axis, which
        they generally do not (mount offset). Returns ``None`` (never a
        guess) if the transform is unavailable.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                scan_header.frame_id, image_header.frame_id, image_header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    scan_header.frame_id, image_header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(
                    "green_box_detector: TF %s->%s unavailable: %s" %
                    (image_header.frame_id, scan_header.frame_id, exc),
                    throttle_duration_sec=2.0)
                return None
        q = tf.transform.rotation
        return _yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _to_output_frame(self, header, x, y):
        """Transform a point ``(x, y, 0)`` in ``header.frame_id`` into
        ``output_frame`` via TF. Returns ``None`` (never a guess) if the
        transform is unavailable - contract requirement: log and skip.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, header.frame_id, header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(
                    "green_box_detector: TF %s->%s unavailable: %s" %
                    (header.frame_id, self.output_frame, exc),
                    throttle_duration_sec=2.0)
                return None
        t = tf.transform.translation
        q = tf.transform.rotation
        rx, ry = _rotate_xy(x, y, q.x, q.y, q.z, q.w)
        return (rx + t.x, ry + t.y)

    def _ray_to_output_frame(self, header, ray_xy):
        """Rotate-only version of ``_to_output_frame`` for a direction
        vector (no translation applied)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, header.frame_id, header.stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.output_frame, header.frame_id, Time())
            except TransformException as exc:
                self.get_logger().warn(
                    "green_box_detector: TF %s->%s unavailable: %s" %
                    (header.frame_id, self.output_frame, exc),
                    throttle_duration_sec=2.0)
                return None
        q = tf.transform.rotation
        return _rotate_xy(ray_xy[0], ray_xy[1], q.x, q.y, q.z, q.w)

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

    def _publish_debug_image(self, bgr, mask, bbox, detected, source):
        debug = bgr.copy()
        outline = cv2.bitwise_and(bgr, bgr, mask=mask)
        debug = cv2.addWeighted(debug, 0.6, outline, 0.4, 0)
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 0, 255), 2)
        text = "detected=%s source=%s" % (detected, source)
        cv2.putText(debug, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 1, cv2.LINE_AA)
        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        except Exception as exc:                      # noqa: BLE001 - log and drop
            self.get_logger().warn("green_box_detector: debug image convert failed: %s" % exc)
            return
        self.debug_image_pub.publish(msg)


def _yaw_from_quaternion(qx, qy, qz, qw):
    """Standard quaternion -> yaw (rotation about z) extraction. Used as an
    approximation of the horizontal-plane rotation between two frames even
    when the transform also carries a small pitch/roll (e.g. camera mount
    tilt) - good enough for bearing matching in this demo scope, not exact
    for a heavily tilted sensor (see README limits).
    """
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _rotate_xy(x, y, qx, qy, qz, qw):
    """Rotate planar point/direction ``(x, y)`` by a frame transform's
    quaternion, using only its yaw component (see ``_yaw_from_quaternion``)
    since every geometry quantity here lives in a horizontal xy-plane.
    Reimplemented without ``tf2_geometry_msgs`` since only that plane is
    needed.
    """
    angle = _yaw_from_quaternion(qx, qy, qz, qw)
    c, s = math.cos(angle), math.sin(angle)
    return (c * x - s * y, s * x + c * y)


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
