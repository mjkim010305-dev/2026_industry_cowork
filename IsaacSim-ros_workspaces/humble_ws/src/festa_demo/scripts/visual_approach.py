#!/usr/bin/env python3
"""/visual_approach: drive up to the green box from the camera image alone.

Replaces FinalApproachStop in bt/festa_demo.xml (called through the
SweepObstacle BT node, same festa_demo/action/Sweep type, result "SUCCESS" or
"ERROR"). On the real robot (2026-10-01) the map-frame box pose that
FinalApproachStop steers by depends on stamped TF lookups, which lag on the Pi,
and the robot stopped up to 15 deg off the box so the sweep hit the wall.

Input: green_box/image_target from detector_node.py, per frame:
    x = (bbox centre column - cx) / fx   (tan of the bearing, + = right)
    y = bbox width / fx                  (angular width)
    z = 1 if the bbox touches the left/right image border

The camera sits off the robot centre (TF base_link -> camera_frame, read once:
0.073 m forward / 0.070 m right on the robot), so "box in the middle of the
image" is not "box straight ahead". The box's range from its known width
moves the measurement to base_link:
    D  = box_width / y                    front face, along the optical axis
    bx = cam_x + D + box_depth / 2        box centre, forward
    by = cam_y - x * D                    box centre, left
Phases:
    1. image: turn in place until |atan2(by, bx)| < align_tol, then creep
       forward while still steering, until the box centre is handoff_dist
       away (the box is still well inside the frame there; at the sweep
       stop its face is ~7 cm from the lens and does not fit).
    2. final: drive straight by odometry for the remaining
       (distance - stop_dist), no camera.
    A forward lidar cone (backstop_dist) stops either phase.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from festa_demo.action import Sweep


class VisualApproach(Node):

    def __init__(self):
        super().__init__('visual_approach')
        p = {n: self.declare_parameter(n, d).value for n, d in (
            ('camera_frame', 'camera_color_optical_frame'), ('base_frame', 'base_link'),
            ('box_width', 0.185), ('box_depth', 0.185),
            ('stop_dist', 0.24), ('handoff_dist', 0.45),
            ('align_tol', 0.035), ('kp', 1.5), ('max_w', 0.6), ('min_w', 0.15),
            ('approach_speed', 0.08), ('image_timeout', 1.0), ('lost_timeout', 3.0),
            ('backstop_dist', 0.10), ('front_half_angle', 0.26), ('time_allowance', 60.0))}
        self.p = p
        cb = ReentrantCallbackGroup()
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.target = None          # (time, x, y, clipped)
        self.odom_xy = None
        self.front_min = None
        self.create_subscription(PointStamped, 'green_box/image_target', self._on_target,
                                 best_effort, callback_group=cb)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan, best_effort, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cam = None
        ActionServer(self, Sweep, 'visual_approach', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info('visual_approach ready')

    def _on_target(self, m):
        with self.lock:
            self.target = (time.monotonic(), m.point.x, m.point.y, m.point.z > 0.5)

    def _on_odom(self, m):
        with self.lock:
            self.odom_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _on_scan(self, m):
        best = None
        for i, r in enumerate(m.ranges):
            a = math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                           math.cos(m.angle_min + i * m.angle_increment))
            if abs(a) <= self.p['front_half_angle'] and math.isfinite(r) and r > 0.0:
                best = r if best is None else min(best, r)
        with self.lock:
            self.front_min = best

    def _camera_offset(self):
        if self.cam is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.p['base_frame'], self.p['camera_frame'], rclpy.time.Time()).transform.translation
                self.cam = (t.x, t.y)
                self.get_logger().info(f'camera at x={t.x:.3f} y={t.y:.3f} in {self.p["base_frame"]}')
            except Exception as exc:  # noqa: BLE001 - retried next tick
                self.get_logger().warn(f'camera TF not yet available: {exc}', throttle_duration_sec=2.0)
        return self.cam

    def _box_in_base(self, x_tan, w_tan):
        cam_x, cam_y = self.cam
        d = self.p['box_width'] / max(w_tan, 1e-3)
        return cam_x + d + self.p['box_depth'] / 2.0, cam_y - x_tan * d

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def _turn(self, bearing):
        w = max(-self.p['max_w'], min(self.p['max_w'], self.p['kp'] * bearing))
        if abs(w) < self.p['min_w']:
            w = math.copysign(self.p['min_w'], w)
        return w

    def _execute(self, goal):
        p = self.p
        fb = Sweep.Feedback()
        result = Sweep.Result()
        start = time.monotonic()
        last_seen = start
        phase, remaining, odom0 = 'image', None, None
        code = 'ERROR'
        while rclpy.ok():
            time.sleep(0.05)
            now = time.monotonic()
            if goal.is_cancel_requested:
                self._cmd(0, 0)
                goal.canceled()
                result.result_code = 'ERROR'
                return result
            if now - start > p['time_allowance']:
                self.get_logger().error('visual_approach: time allowance exceeded')
                break
            with self.lock:
                target, odom_xy, front_min = self.target, self.odom_xy, self.front_min
            if front_min is not None and front_min <= p['backstop_dist']:
                self.get_logger().info(f'visual_approach: lidar backstop at {front_min:.2f} m')
                code = 'SUCCESS'
                break
            if phase == 'final':
                if odom_xy is None:
                    continue
                if odom0 is None:
                    odom0 = odom_xy
                done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
                if done >= remaining:
                    self.get_logger().info(f'visual_approach: final {done:.2f}/{remaining:.2f} m - stopped')
                    code = 'SUCCESS'
                    break
                self._cmd(p['approach_speed'], 0.0)
                continue
            if self._camera_offset() is None:
                self._cmd(0, 0)
                continue
            if target is None or now - target[0] > p['image_timeout'] or target[3]:
                self._cmd(0, 0)
                if now - last_seen > p['lost_timeout']:
                    self.get_logger().error('visual_approach: box not in view')
                    break
                continue
            last_seen = now
            bx, by = self._box_in_base(target[1], target[2])
            bearing, dist = math.atan2(by, bx), math.hypot(bx, by)
            aligned = abs(bearing) < p['align_tol']
            if not aligned:
                fb.state = f'ALIGN bearing={math.degrees(bearing):.1f}deg dist={dist:.2f}m'
                self._cmd(0.0, self._turn(bearing))
            elif dist <= p['handoff_dist']:
                phase, remaining = 'final', max(0.0, dist - p['stop_dist'])
                fb.state = f'FINAL straight {remaining:.2f}m'
                self.get_logger().info(
                    f'visual_approach: aligned at {math.degrees(bearing):.1f} deg, box {dist:.2f} m '
                    f'-> straight {remaining:.2f} m by odometry')
                self._cmd(0, 0)
            else:
                fb.state = f'APPROACH bearing={math.degrees(bearing):.1f}deg dist={dist:.2f}m'
                self._cmd(p['approach_speed'], self.p['kp'] * bearing)
            goal.publish_feedback(fb)
        self._cmd(0, 0)
        result.result_code = code
        if code == 'SUCCESS':
            goal.succeed()
        else:
            goal.abort()
        return result


def main():
    rclpy.init()
    node = VisualApproach()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node._cmd(0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
