#!/usr/bin/env python3
"""/push_through: after the sweep, drive straight ahead a short distance so the
robot's body shoves the partly swept box out of the way (user, 2026-10-01:
"if the sweep has pushed it some way, just push the rest with the body").

Called from bt/festa_demo.xml right after the sweep, through the SweepObstacle
BT node (festa_demo/action/Sweep, result "SUCCESS" / "ERROR"). Straight at
`speed` until odometry has covered `distance`; a forward lidar cone stops early
at `wall_stop` (the low box itself is under the lidar plane, so this only
reacts to walls).
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from festa_demo.action import Sweep


class PushThrough(Node):

    def __init__(self):
        super().__init__('push_through')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            ('distance', 0.35), ('speed', 0.08), ('wall_stop', 0.15),
            ('front_half_angle', 0.26), ('time_allowance', 15.0))}
        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.odom_xy = None
        self.front_min = None
        self._cone_cache = None  # (angle_min, angle_increment, n, indices) - festa_demo: Pi load, 2026-10-01
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, Sweep, 'push_through', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info('push_through ready')

    def _on_odom(self, m):
        with self.lock:
            self.odom_xy = (m.pose.pose.position.x, m.pose.pose.position.y)

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

    def _cmd(self, v):
        t = Twist()
        t.linear.x = float(v)
        self.cmd_pub.publish(t)

    def _execute(self, goal):
        p = self.p
        result = Sweep.Result()
        start = time.monotonic()
        odom0 = None
        code = 'ERROR'
        while rclpy.ok():
            time.sleep(0.05)
            if goal.is_cancel_requested:
                self._cmd(0)
                goal.canceled()
                result.result_code = 'ERROR'
                return result
            if time.monotonic() - start > p['time_allowance']:
                self.get_logger().error('push_through: time allowance exceeded')
                break
            with self.lock:
                odom_xy, front_min = self.odom_xy, self.front_min
            if odom_xy is None:
                continue
            if odom0 is None:
                odom0 = odom_xy
            done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
            if front_min is not None and front_min <= p['wall_stop']:
                self.get_logger().info(f'push_through: wall at {front_min:.2f} m after {done:.2f} m - stopped')
                code = 'SUCCESS'
                break
            if done >= p['distance']:
                self.get_logger().info(f'push_through: pushed {done:.2f} m - done')
                code = 'SUCCESS'
                break
            self._cmd(p['speed'])
        self._cmd(0)
        result.result_code = code
        if code == 'SUCCESS':
            goal.succeed()
        else:
            goal.abort()
        return result


def main():
    rclpy.init()
    node = PushThrough()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    finally:
        node._cmd(0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
