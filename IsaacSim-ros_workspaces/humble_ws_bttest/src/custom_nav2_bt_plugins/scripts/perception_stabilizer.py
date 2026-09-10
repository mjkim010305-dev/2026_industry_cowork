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

"""Perception Stabilizer node.

Independent, always-on ROS 2 node that turns the flaky VLM detect signal plus a
raw RealSense depth stream into a *stabilised* obstacle estimate for the Nav2
behavior tree. It implements the design in
``custom_nav2_bt_plugins/bt_perception_stabilizer_design.md``:

  * 3-state machine  IDLE <-> CANDIDATE <-> TRACKING  with time-window
    hysteresis, so momentary VLM false/true flips never reach the BT.
  * a rolling point-cloud buffer (last ``buffer_duration`` seconds of depth
    points in the ROI) that is voxel-downsampled and fed to a *single* PCA -
    not per-frame yaw averaging - to get the obstacle centroid and surface
    normal.
  * convergence test: circular std-dev of the normal yaw over the window; below
    ``converge_std_deg`` the estimate is "stable".
  * missed-frame extrapolation: while TRACKING with a bad frame, the last good
    pose is held.
  * ``busy`` flag + depth ``queue_size = 1``: overlapping work is dropped, the
    node always processes the freshest frame it can.

Outputs (consumed by the thin BT nodes ``IsObstacleTracking`` /
``LockApproachPose``):

  * ``/perception_state``          std_msgs/String   IDLE | CANDIDATE | TRACKING
  * ``/obstacle_pose_stable``      geometry_msgs/PoseStamped  (map frame,
                                   orientation = surface-normal yaw), published
                                   only while TRACKING
  * ``/obstacle_pose_confidence``  std_msgs/Float32  0..1

The VLM ROI topic does not exist yet, so the ROI defaults to a centred ratio
crop of the depth image (like DepthCentroidLocalizer). Point ``roi_topic`` at
the real ``sensor_msgs/RegionOfInterest`` topic once the teammate publishes it -
no other change needed.
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_system_default

from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException


IDLE = "IDLE"
CANDIDATE = "CANDIDATE"
TRACKING = "TRACKING"


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def rotate_vec_by_quat(q, v):
    """Rotate 3-vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return np.array([rx, ry, rz], dtype=np.float64)


def circular_std(angles):
    """Circular standard deviation [rad] of a 1-D array of angles."""
    if len(angles) < 2:
        return float("inf")
    s = np.mean(np.sin(angles))
    c = np.mean(np.cos(angles))
    r = math.hypot(s, c)
    if r <= 1e-9:
        return float("inf")
    return math.sqrt(max(0.0, -2.0 * math.log(r)))


class PerceptionStabilizer(Node):

    def __init__(self):
        super().__init__("perception_stabilizer")

        # --- topics -------------------------------------------------------
        self.depth_topic = self.declare_parameter("depth_topic", "/depth_camera").value
        self.info_topic = self.declare_parameter("camera_info_topic", "/depth_camera_info").value
        self.detect_topic = self.declare_parameter("detect_topic", "/obstacle/is_movable").value
        self.roi_topic = self.declare_parameter("roi_topic", "").value
        self.target_frame = self.declare_parameter("target_frame", "map").value
        self.optical_frame = self.declare_parameter("optical_frame", "").value

        self.pose_out_topic = self.declare_parameter(
            "pose_out_topic", "/obstacle_pose_stable").value
        self.state_out_topic = self.declare_parameter(
            "state_out_topic", "/perception_state").value
        self.conf_out_topic = self.declare_parameter(
            "confidence_out_topic", "/obstacle_pose_confidence").value

        # --- ROI / depth gating ----------------------------------------
        self.roi_w_ratio = self.declare_parameter("roi_width_ratio", 0.4).value
        self.roi_h_ratio = self.declare_parameter("roi_height_ratio", 0.6).value
        self.min_range = self.declare_parameter("min_range", 0.2).value
        self.max_range = self.declare_parameter("max_range", 6.0).value

        # --- buffer / PCA --------------------------------------------------
        self.process_rate = self.declare_parameter("process_rate", 10.0).value
        self.buffer_duration = self.declare_parameter("buffer_duration", 1.0).value
        self.max_buffer_points = int(self.declare_parameter("max_buffer_points", 20000).value)
        self.max_new_points = int(self.declare_parameter("max_new_points_per_frame", 3000).value)
        self.voxel_size = self.declare_parameter("voxel_size", 0.02).value
        self.min_points = int(self.declare_parameter("min_points", 300).value)
        self.plane_residual_max = self.declare_parameter("plane_residual_max", 0.05).value

        # --- state machine ----------------------------------------------
        self.state_window = self.declare_parameter("state_window", 1.0).value
        self.detect_timeout = self.declare_parameter("detect_timeout", 2.0).value
        self.tracking_enter_rate = self.declare_parameter("tracking_enter_rate", 0.7).value
        self.converge_std_deg = self.declare_parameter("converge_std_deg", 5.0).value
        self.tracking_lost_timeout = self.declare_parameter("tracking_lost_timeout", 0.7).value
        self.candidate_timeout = self.declare_parameter("candidate_timeout", 3.0).value
        self.transform_timeout = self.declare_parameter("transform_timeout", 0.2).value
        self.data_timeout = self.declare_parameter("data_timeout", 1.0).value
        self.debug = self.declare_parameter("debug", False).value

        # --- runtime state --------------------------------------------
        self.state = IDLE
        self.candidate_since = None
        self.last_detect_true_t = None
        self.busy = False

        self.last_depth = None
        self.last_depth_t = None
        self.last_info = None
        self.detect_hist = deque()          # (t, bool)
        self.last_roi = None                # (x, y, w, h) pixels

        self.buf_pts = np.zeros((0, 3), dtype=np.float32)
        self.buf_t = np.zeros((0,), dtype=np.float64)
        self.yaw_hist = deque()            # (t, yaw_map)

        self.last_good_pose = None          # (x, y, yaw) in map
        self.last_good_t = None

        # --- ROS I/O ----------------------------------------------------
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, sensor_qos)
        self.create_subscription(Bool, self.detect_topic, self._on_detect, 10)
        if self.roi_topic:
            self.create_subscription(RegionOfInterest, self.roi_topic, self._on_roi, 10)

        self.pose_pub = self.create_publisher(
            PoseStamped, self.pose_out_topic, qos_profile_system_default)
        self.state_pub = self.create_publisher(
            String, self.state_out_topic, qos_profile_system_default)
        self.conf_pub = self.create_publisher(
            Float32, self.conf_out_topic, qos_profile_system_default)

        period = 1.0 / max(1.0, self.process_rate)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "perception_stabilizer: depth='%s' info='%s' detect='%s' roi='%s' -> "
            "'%s' / '%s' / '%s' in '%s' @ %.1f Hz (%s ROI)" % (
                self.depth_topic, self.info_topic, self.detect_topic,
                self.roi_topic or "(centre crop)", self.state_out_topic,
                self.pose_out_topic, self.conf_out_topic, self.target_frame,
                self.process_rate, "topic" if self.roi_topic else "centre-crop"))

    # ------------------------------------------------------------------
    # subscriptions
    # ------------------------------------------------------------------
    def _on_depth(self, msg):
        self.last_depth = msg
        self.last_depth_t = self.get_clock().now()

    def _on_info(self, msg):
        self.last_info = msg

    def _on_detect(self, msg):
        now = self.get_clock().now()
        self.detect_hist.append((now, bool(msg.data)))
        if msg.data:
            self.last_detect_true_t = now
        self._trim_detect(now)

    def _on_roi(self, msg):
        self.last_roi = (int(msg.x_offset), int(msg.y_offset), int(msg.width), int(msg.height))

    def _trim_detect(self, now):
        horizon = now.nanoseconds * 1e-9 - self.state_window
        while self.detect_hist and self.detect_hist[0][0].nanoseconds * 1e-9 < horizon:
            self.detect_hist.popleft()

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def _tick(self):
        if self.busy:
            return
        self.busy = True
        try:
            self._process()
        except Exception as exc:  # never let the timer die
            self.get_logger().error("perception_stabilizer tick failed: %r" % exc)
        finally:
            self.busy = False

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _detect_fresh_true(self):
        if self.last_detect_true_t is None:
            return False
        age = self._now_s() - self.last_detect_true_t.nanoseconds * 1e-9
        return age <= self.detect_timeout

    def _detect_rate(self):
        if not self.detect_hist:
            return 0.0
        hits = sum(1 for _, d in self.detect_hist if d)
        return hits / float(len(self.detect_hist))

    def _process(self):
        now_s = self._now_s()
        self._trim_detect(self.get_clock().now())

        detect_true = self._detect_fresh_true()
        detect_rate = self._detect_rate()
        lost_for = (
            float("inf") if self.last_detect_true_t is None
            else now_s - self.last_detect_true_t.nanoseconds * 1e-9)

        # ---- state transitions (detection side) -----------------------
        if self.state == IDLE:
            if detect_true:
                self._enter_candidate(now_s)
        elif self.state == CANDIDATE:
            if lost_for > self.candidate_timeout:
                self._enter_idle()
        elif self.state == TRACKING:
            if lost_for > self.tracking_lost_timeout:
                self._enter_candidate(now_s)

        if self.state == IDLE:
            # Skip all heavy work while idle (design doc section 6).
            self._reset_buffers()
            self._publish_state()
            return

        # ---- perception: accumulate + PCA ---------------------------
        est = self._estimate()   # dict or None

        if est is not None and est["planar"]:
            self.yaw_hist.append((now_s, est["yaw_map"]))
        while self.yaw_hist and self.yaw_hist[0][0] < now_s - self.state_window:
            self.yaw_hist.popleft()

        yaw_std = circular_std(np.array([y for _, y in self.yaw_hist]))
        converged = math.degrees(yaw_std) < self.converge_std_deg

        # ---- state transition (convergence side) --------------------
        if self.state == CANDIDATE:
            if (detect_rate >= self.tracking_enter_rate and converged and
                    est is not None and est["n_points"] >= self.min_points):
                self.state = TRACKING
                self.get_logger().info(
                    "perception_stabilizer: CANDIDATE -> TRACKING "
                    "(detect_rate=%.2f yaw_std=%.1f deg)" % (detect_rate, math.degrees(yaw_std)))

        # ---- output -------------------------------------------------
        self._publish_state()

        if self.state != TRACKING:
            return

        if est is not None and est["planar"]:
            pose = (est["cx_map"], est["cy_map"], est["yaw_map"])
            self.last_good_pose = pose
            self.last_good_t = now_s
        elif self.last_good_pose is not None and (now_s - self.last_good_t) <= self.buffer_duration:
            pose = self.last_good_pose   # extrapolate: hold last good
        else:
            return   # tracking but nothing trustworthy yet

        conf = self._confidence(detect_rate, yaw_std, est)
        self._publish_pose(pose, conf)

    # ------------------------------------------------------------------
    # state helpers
    # ------------------------------------------------------------------
    def _enter_idle(self):
        if self.state != IDLE:
            self.get_logger().info("perception_stabilizer: %s -> IDLE" % self.state)
        self.state = IDLE
        self.candidate_since = None

    def _enter_candidate(self, now_s):
        if self.state != CANDIDATE:
            self.get_logger().info("perception_stabilizer: %s -> CANDIDATE" % self.state)
        self.state = CANDIDATE
        self.candidate_since = now_s

    def _reset_buffers(self):
        self.buf_pts = np.zeros((0, 3), dtype=np.float32)
        self.buf_t = np.zeros((0,), dtype=np.float64)
        self.yaw_hist.clear()
        self.last_good_pose = None
        self.last_good_t = None

    # ------------------------------------------------------------------
    # perception
    # ------------------------------------------------------------------
    def _depth_to_metres(self, msg):
        enc = msg.encoding
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("32FC1", "32FC"):
            arr = buf.view(np.float32).reshape(msg.height, msg.width).astype(np.float32)
        elif enc in ("16UC1", "mono16"):
            arr = buf.view(np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 1e-3
        else:
            raise ValueError("unsupported depth encoding '%s'" % enc)
        return arr

    def _roi_bounds(self, w, h):
        if self.last_roi is not None:
            x, y, rw, rh = self.last_roi
            x0 = max(0, min(w - 1, x))
            y0 = max(0, min(h - 1, y))
            x1 = max(x0 + 1, min(w, x + rw))
            y1 = max(y0 + 1, min(h, y + rh))
            return x0, y0, x1, y1
        cw = int(w * self.roi_w_ratio)
        ch = int(h * self.roi_h_ratio)
        x0 = (w - cw) // 2
        y0 = (h - ch) // 2
        return x0, y0, x0 + cw, y0 + ch

    def _estimate(self):
        """Deproject this ROI, add to the rolling buffer, PCA the buffer."""
        if self.last_depth is None or self.last_info is None:
            return None
        if (self.get_clock().now() - self.last_depth_t).nanoseconds * 1e-9 > self.data_timeout:
            self.get_logger().warn("perception_stabilizer: stale depth", throttle_duration_sec=5.0)
            return None

        msg = self.last_depth
        try:
            depth = self._depth_to_metres(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return None

        k = np.array(self.last_info.k, dtype=np.float64).reshape(3, 3)
        fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
        if fx <= 0.0 or fy <= 0.0:
            return None

        h, w = depth.shape
        x0, y0, x1, y1 = self._roi_bounds(w, h)
        sub = depth[y0:y1, x0:x1]
        us = np.arange(x0, x1)
        vs = np.arange(y0, y1)
        uu, vv = np.meshgrid(us, vs)

        z = sub.reshape(-1)
        uu = uu.reshape(-1)
        vv = vv.reshape(-1)
        good = np.isfinite(z) & (z > self.min_range) & (z < self.max_range)
        if np.count_nonzero(good) < 20:
            return None
        z = z[good]
        uu = uu[good].astype(np.float64)
        vv = vv[good].astype(np.float64)
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        if pts.shape[0] > self.max_new_points:
            idx = np.random.choice(pts.shape[0], self.max_new_points, replace=False)
            pts = pts[idx]

        # append to rolling buffer, trim by time then by size
        now_s = self._now_s()
        self.buf_pts = np.vstack([self.buf_pts, pts])
        self.buf_t = np.concatenate([self.buf_t, np.full(pts.shape[0], now_s)])
        keep = self.buf_t > (now_s - self.buffer_duration)
        self.buf_pts = self.buf_pts[keep]
        self.buf_t = self.buf_t[keep]
        if self.buf_pts.shape[0] > self.max_buffer_points:
            self.buf_pts = self.buf_pts[-self.max_buffer_points:]
            self.buf_t = self.buf_t[-self.max_buffer_points:]

        # voxel downsample the accumulated cloud (bounds PCA cost, evens density)
        p = self.buf_pts.astype(np.float64)
        if self.voxel_size > 0.0 and p.shape[0] > 0:
            keys = np.floor(p / self.voxel_size).astype(np.int64)
            _, uniq = np.unique(keys, axis=0, return_index=True)
            p = p[np.sort(uniq)]

        n_points = p.shape[0]
        if n_points < self.min_points:
            return {"planar": False, "n_points": n_points}

        centroid = p.mean(axis=0)
        d = p - centroid
        cov = (d.T @ d) / float(n_points)
        evals, evecs = np.linalg.eigh(cov)   # ascending
        normal = evecs[:, 0]
        residual = math.sqrt(max(0.0, evals[0]))
        # planar iff smallest spread is small both absolutely and vs the plane
        planar = (residual < self.plane_residual_max and
                  evals[0] < 0.1 * max(evals[1], 1e-9))

        # orient normal towards the camera (origin in optical frame)
        if float(np.dot(normal, centroid)) > 0.0:
            normal = -normal

        yaw_map, cx_map, cy_map = self._to_map(msg.header.frame_id, centroid, normal)
        if yaw_map is None:
            return {"planar": False, "n_points": n_points}

        return {
            "planar": bool(planar),
            "n_points": n_points,
            "residual": residual,
            "yaw_map": yaw_map,
            "cx_map": cx_map,
            "cy_map": cy_map,
        }

    def _to_map(self, depth_frame, centroid_cam, normal_cam):
        frame = self.optical_frame or depth_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                "perception_stabilizer: TF %s <- %s failed: %s" % (
                    self.target_frame, frame, exc),
                throttle_duration_sec=2.0)
            return None, None, None
        t = tf.transform.translation
        r = tf.transform.rotation
        q = (r.x, r.y, r.z, r.w)
        c_map = rotate_vec_by_quat(q, centroid_cam) + np.array([t.x, t.y, t.z])
        n_map = rotate_vec_by_quat(q, normal_cam)   # rotation only, no translation
        yaw = math.atan2(float(n_map[1]), float(n_map[0]))
        return yaw, float(c_map[0]), float(c_map[1])

    def _confidence(self, detect_rate, yaw_std, est):
        w_detect = max(0.0, min(1.0, detect_rate))
        ref = math.radians(max(1e-3, self.converge_std_deg))
        w_conv = math.exp(-yaw_std / ref) if math.isfinite(yaw_std) else 0.0
        w_plane = 1.0
        if est is not None and "residual" in est:
            w_plane = max(0.0, min(1.0, 1.0 - est["residual"] / max(1e-6, self.plane_residual_max)))
        return float(max(0.0, min(1.0, w_detect * w_conv * w_plane)))

    # ------------------------------------------------------------------
    # publishing
    # ------------------------------------------------------------------
    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def _publish_pose(self, pose, conf):
        x, y, yaw = pose
        qx, qy, qz, qw = quat_from_yaw(yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)
        self.conf_pub.publish(Float32(data=conf))
        if self.debug:
            self.get_logger().info(
                "perception_stabilizer: TRACKING pose (%.2f, %.2f) yaw=%.1f deg conf=%.2f" % (
                    x, y, math.degrees(yaw), conf))


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
