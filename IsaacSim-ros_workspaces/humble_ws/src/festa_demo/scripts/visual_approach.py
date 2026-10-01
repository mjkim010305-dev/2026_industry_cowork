#!/usr/bin/env python3
"""/visual_approach: drive up to the green box by the camera image alone.

Replaces FinalApproachStop in bt/festa_demo.xml (called through the
SweepObstacle BT node, same festa_demo/action/Sweep type, result "SUCCESS" or
"ERROR").

User rule (real robot, 2026-10-01): the sweep works when the box sits in the
middle of the camera image and fills almost the whole frame (82.9 % green in
the reference view); a box off to one side does not get swept. So:
    - steer the whole way to keep the box centroid in the middle of the image;
    - far (green < near_fill of the frame): drive at far_speed with proportional
      steering, turn in place only beyond far_align_tol (user, 2026-10-01: the
      3 deg stop-and-turn zigzagged slowly all the way in);
    - near: approach_speed, turn in place while more than align_tol off;
    - stop once green covers fill_stop of the frame and the box is centred.
No range estimate, TF or odometry. A forward lidar cone (backstop_dist) still
stops the robot.

Inputs from detector_node.py:
    green_box/image_target  x = (bbox centre column - cx) / fx  (tan, + = right)
    green_box/image_fill    green pixels / frame pixels
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from festa_demo.action import Sweep


class VisualApproach(Node):

    def __init__(self):
        super().__init__('visual_approach')
        p = {n: self.declare_parameter(n, d).value for n, d in (
            ('fill_stop', 0.80), ('align_tol', 0.05), ('kp', 1.5), ('max_w', 0.6), ('min_w', 0.15),
            ('approach_speed', 0.05), ('far_speed', 0.12), ('near_fill', 0.30),
            ('far_align_tol', 0.26), ('image_timeout', 1.0), ('lost_timeout', 3.0),
            ('backstop_dist', 0.10), ('front_half_angle', 0.26), ('time_allowance', 60.0))}
        self.p = p
        cb = ReentrantCallbackGroup()
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.target = None          # (time, x_tan)
        self.fill = None            # (time, fraction)
        self.front_min = None
        self._cone_cache = None  # (angle_min, angle_increment, n, indices) - festa_demo: Pi load, 2026-10-01
        self.create_subscription(PointStamped, 'green_box/image_target', self._on_target,
                                 best_effort, callback_group=cb)
        self.create_subscription(Float32, 'green_box/image_fill', self._on_fill,
                                 best_effort, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan, best_effort, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, Sweep, 'visual_approach', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info('visual_approach ready')

    def _on_target(self, m):
        with self.lock:
            self.target = (time.monotonic(), m.point.x)

    def _on_fill(self, m):
        with self.lock:
            self.fill = (time.monotonic(), m.data)

    def _on_scan(self, m):
        # festa_demo: Pi load, 2026-10-01 - the set of beam indices inside the
        # front cone only depends on the scan geometry, not the ranges, so
        # cache it instead of recomputing atan2(sin, cos) per beam per message.
        key = (m.angle_min, m.angle_increment, len(m.ranges))
        if self._cone_cache is None or self._cone_cache[0] != key:
            half = self.p['front_half_angle']
            indices = [i for i in range(len(m.ranges))
                       if abs(math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                                        math.cos(m.angle_min + i * m.angle_increment))) <= half]
            self._cone_cache = (key, indices)
        best = None
        for i in self._cone_cache[1]:
            r = m.ranges[i]
            if math.isfinite(r) and r > 0.0:
                best = r if best is None else min(best, r)
        with self.lock:
            self.front_min = best

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def _execute(self, goal):
        p = self.p
        fb = Sweep.Feedback()
        result = Sweep.Result()
        start = time.monotonic()
        last_seen = start
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
                target, fill, front_min = self.target, self.fill, self.front_min
            if front_min is not None and front_min <= p['backstop_dist']:
                self.get_logger().info(f'visual_approach: lidar backstop at {front_min:.2f} m')
                code = 'SUCCESS'
                break
            if target is None or now - target[0] > p['image_timeout']:
                self._cmd(0, 0)
                if now - last_seen > p['lost_timeout']:
                    self.get_logger().error('visual_approach: box not in view')
                    break
                continue
            last_seen = now
            bearing = -math.atan(target[1])            # + = box left of the image centre
            fill_now = fill[1] if fill is not None and now - fill[0] <= p['image_timeout'] else 0.0
            near = fill_now >= p['near_fill']
            filled = fill_now >= p['fill_stop']
            if abs(bearing) > (p['align_tol'] if near else p['far_align_tol']):
                w = max(-p['max_w'], min(p['max_w'], p['kp'] * bearing))
                self._cmd(0.0, math.copysign(max(abs(w), p['min_w']), w))
                fb.state = f'CENTRE bearing={math.degrees(bearing):.1f}deg'
            elif filled:
                self.get_logger().info(
                    f'visual_approach: box fills {100 * fill[1]:.0f}% of the frame, '
                    f'centred at {math.degrees(bearing):.1f} deg - stopped')
                code = 'SUCCESS'
                break
            else:
                w = max(-p['max_w'], min(p['max_w'], p['kp'] * bearing))
                self._cmd(p['approach_speed'] if near else p['far_speed'], w)
                fb.state = (f'APPROACH{"" if near else "-FAR"} fill={100 * fill_now:.0f}% '
                            f'bearing={math.degrees(bearing):.1f}deg')
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
