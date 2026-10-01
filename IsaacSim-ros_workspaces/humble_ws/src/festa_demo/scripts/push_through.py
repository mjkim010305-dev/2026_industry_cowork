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
            # festa_demo (2026-10-02, sim S1m): wall_stop is lidar range; the bumper is ~0.10 m ahead
            # of base_scan, so 0.15 left ~5 cm and the robot pushed into a wall for 60 s. Now 0.25.
            ('distance', 0.35), ('speed', 0.08), ('wall_stop', 0.25),
            ('front_half_angle', 0.26), ('time_allowance', 15.0),
            ('stall_s', 3.0), ('stall_dist', 0.02))}
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.odom_xy = None
        self.front_min = None
        self._goal_gen = 0  # festa_demo: preemption, 2026-10-01 - only the newest goal may drive
        self._cone_cache = None  # (angle_min, angle_increment, n, indices) - festa_demo: Pi load, 2026-10-01
        self.odom_sub = None  # festa_demo: idle CPU, 2026-10-01 - live only while a goal is executing
        self.scan_sub = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, Sweep, 'push_through', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.cb)
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
            if math.isfinite(r) and r > 0.0 and m.range_min <= r <= m.range_max:  # festa_demo: S1e 0.02 m glitch < range_min 0.12 (real LDS-02 reports range_min 0.0)
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
        progress = (start, 0.0)     # festa_demo (S25_V1): (time, distance) of the last 2 cm gained
        with self.lock:
            self.odom_xy = None
            self.front_min = None
            self._goal_gen += 1
            my = self._goal_gen
        # festa_demo: idle CPU, 2026-10-01 - measured 33% of a core idle, mostly
        # /odom at ~49 Hz; only subscribe while a goal is actually running.
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10,
                                                   callback_group=self.cb)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._on_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.cb)
        try:
            while rclpy.ok():
                time.sleep(0.05)
                with self.lock:
                    preempted = self._goal_gen != my
                if preempted:
                    # festa_demo: preemption, 2026-10-01 - a newer goal owns the
                    # base now; do not touch cmd_vel, just drop out.
                    self.get_logger().warn('push_through: preempted by a newer goal')
                    goal.abort()
                    result.result_code = 'ERROR'
                    return result
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
                # festa_demo (2026-10-02, sim S25_V1): wedged against a pillar, the push drove
                # 60 s for 1 cm. Stop when stall_dist is not gained within stall_s.
                if done - progress[1] >= p['stall_dist']:
                    progress = (time.monotonic(), done)
                elif time.monotonic() - progress[0] > p['stall_s']:
                    self.get_logger().warn(f'push_through: no progress for {p["stall_s"]:.0f} s after '
                                           f'{done:.2f} m - stopped')
                    break
                self._cmd(p['speed'])
            self._cmd(0)
            result.result_code = code
            if code == 'SUCCESS':
                goal.succeed()
            else:
                goal.abort()
            return result
        finally:
            self.destroy_subscription(self.odom_sub)
            self.destroy_subscription(self.scan_sub)
            self.odom_sub = None
            self.scan_sub = None


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
