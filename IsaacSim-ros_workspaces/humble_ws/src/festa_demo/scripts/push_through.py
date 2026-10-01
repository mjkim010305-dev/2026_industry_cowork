#!/usr/bin/env python3
"""/push_through: after the sweep, drive straight ahead a short distance so the
robot's body shoves the partly swept box out of the way (user, 2026-10-01:
"if the sweep has pushed it some way, just push the rest with the body").

Called from bt/festa_demo.xml right after the sweep, through the SweepObstacle
BT node (festa_demo/action/Sweep, result "SUCCESS" / "ERROR"). Straight at
`speed` until odometry has covered `distance`; a forward lidar cone stops early
at `wall_stop` (the low box itself is under the lidar plane, so this only
reacts to walls).

festa_demo (2026-10-02, sim S25_V5): the lidar stop missed a wall and the push ended
0.15 m from it, inside Nav2's 0.20 m footprint; every Nav2 motion (FollowPath, BackUp,
Spin) was then rejected as "in collision" and the robot never moved again. So:
  - the push also stops on the map: centre clearance < map_stop or the point
    look_ahead m ahead closer than map_stop_ahead to an occupied /map cell;
  - /escape (same node): if the centre is closer than map_stop to a wall, drive straight
    (no costmap check) toward the side - forward or back - with more clearance, until
    escape_target, escape_max m, stalled, or the leading edge gets within escape_edge_clear.
Robot pose in the map: green_box/map_pose from box_on_path.py (AMCL + odometry).
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
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSReliabilityPolicy
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
            ('stall_s', 3.0), ('stall_dist', 0.02),
            # map_stop: distance-field value (cell centres) at which Nav2's 0.20 m footprint
            # starts touching a wall cell (sim S25_V5 deadlock at 0.20; the sim course passes
            # 0.22 from the pillar, so no margin above that)
            ('map_stop', 0.21), ('escape_target', 0.25), ('look_ahead', 0.15), ('map_stop_ahead', 0.20),
            ('escape_speed', 0.05), ('escape_max', 0.25), ('escape_front', 0.14),
            ('escape_rear', 0.22), ('escape_edge_clear', 0.08), ('pose_timeout', 1.0))}
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.odom_xy = None
        self.front_min = None
        self._goal_gen = 0  # festa_demo: preemption, 2026-10-01 - only the newest goal may drive
        self._cone_cache = None  # (angle_min, angle_increment, n, indices) - festa_demo: Pi load, 2026-10-01
        self.odom_sub = None  # festa_demo: idle CPU, 2026-10-01 - live only while a goal is executing
        self.scan_sub = None
        self.dist = None            # (distance field m, origin x, origin y, resolution)
        self.map_pose = None        # (monotonic time, x, y, yaw)
        self.pose_sub = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            OccupancyGrid, 'map', self._on_map,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE), callback_group=self.cb)
        ActionServer(self, Sweep, 'push_through', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.cb)
        ActionServer(self, Sweep, 'escape', execute_callback=self._escape,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.cb)
        self.get_logger().info('push_through ready')

    def _on_map(self, m):
        # distance (m) from every cell to the nearest occupied or unknown cell, computed once
        import cv2
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        free = ((grid >= 0) & (grid < 50)).astype(np.uint8)
        d = cv2.distanceTransform(free, cv2.DIST_L2, 5) * m.info.resolution
        with self.lock:
            self.dist = (d, m.info.origin.position.x, m.info.origin.position.y, m.info.resolution)

    def _on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.map_pose = (time.monotonic(), m.pose.position.x, m.pose.position.y, yaw)

    def _clear(self, pose, ahead=0.0):
        """Clearance (m) to the nearest wall cell at the point `ahead` m along the heading."""
        d, ox, oy, res = self.dist
        x, y = pose[1] + ahead * math.cos(pose[3]), pose[2] + ahead * math.sin(pose[3])
        i, j = int((x - ox) / res), int((y - oy) / res)
        if not (0 <= j < d.shape[0] and 0 <= i < d.shape[1]):
            return 0.0
        return float(d[j, i])

    def _fresh_pose(self):
        with self.lock:
            pose, ok = self.map_pose, self.dist is not None
        if not ok or pose is None or time.monotonic() - pose[0] > self.p['pose_timeout']:
            return None
        return pose

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
        self.pose_sub = self.create_subscription(PoseStamped, 'green_box/map_pose', self._on_pose, 10,
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
                pose = self._fresh_pose()
                if pose is not None and (self._clear(pose) < p['map_stop']
                                         or self._clear(pose, p['look_ahead']) < p['map_stop_ahead']):
                    self.get_logger().info(
                        f'push_through: map wall {self._clear(pose):.2f} m (ahead '
                        f'{self._clear(pose, p["look_ahead"]):.2f} m) after {done:.2f} m - stopped')
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
            self.destroy_subscription(self.pose_sub)
            self.odom_sub = None
            self.scan_sub = None
            self.pose_sub = None

    def _escape(self, goal):
        p = self.p
        result = Sweep.Result()
        with self.lock:
            self._goal_gen += 1
            my = self._goal_gen
            self.map_pose = None
            self.odom_xy = None
        self.pose_sub = self.create_subscription(PoseStamped, 'green_box/map_pose', self._on_pose, 10,
                                                 callback_group=self.cb)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self._on_odom, 10,
                                                   callback_group=self.cb)
        code = 'ERROR'
        try:
            start = time.monotonic()
            pose = None
            while rclpy.ok() and pose is None and time.monotonic() - start < 2.0:
                time.sleep(0.05)
                pose = self._fresh_pose()
            if pose is None:
                self.get_logger().warn('escape: no map pose - not moving')
            elif self._clear(pose) >= p['map_stop']:
                code = 'SUCCESS'
            else:
                fwd, back = self._clear(pose, p['look_ahead']), self._clear(pose, -p['look_ahead'])
                sign = 1.0 if fwd > back else -1.0
                edge = sign * (p['escape_front'] if sign > 0 else p['escape_rear'])
                self.get_logger().warn(
                    f'escape: {self._clear(pose):.2f} m from a wall (ahead {fwd:.2f}, behind {back:.2f}) '
                    f'- driving {"forward" if sign > 0 else "back"}')
                odom0, progress = None, (time.monotonic(), 0.0)
                while rclpy.ok():
                    time.sleep(0.05)
                    with self.lock:
                        preempted = self._goal_gen != my
                        odom_xy = self.odom_xy
                    if preempted:
                        goal.abort()
                        result.result_code = 'ERROR'
                        return result
                    if goal.is_cancel_requested:
                        self._cmd(0)
                        goal.canceled()
                        result.result_code = 'ERROR'
                        return result
                    pose = self._fresh_pose()
                    if pose is None or odom_xy is None:
                        self._cmd(0)
                        if time.monotonic() - progress[0] > p['stall_s']:
                            break
                        continue
                    if odom0 is None:
                        odom0 = odom_xy
                    done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
                    if self._clear(pose) >= p['escape_target']:
                        self.get_logger().info(f'escape: clear ({self._clear(pose):.2f} m) after {done:.2f} m')
                        code = 'SUCCESS'
                        break
                    if self._clear(pose, edge) < p['escape_edge_clear'] or done >= p['escape_max']:
                        self.get_logger().warn(f'escape: stopped after {done:.2f} m, still '
                                               f'{self._clear(pose):.2f} m from a wall')
                        break
                    if done - progress[1] >= p['stall_dist']:
                        progress = (time.monotonic(), done)
                    elif time.monotonic() - progress[0] > p['stall_s']:
                        self.get_logger().warn(f'escape: no progress after {done:.2f} m')
                        break
                    self._cmd(sign * p['escape_speed'])
                self._cmd(0)
            result.result_code = code
            if code == 'SUCCESS':
                goal.succeed()
            else:
                goal.abort()
            return result
        finally:
            self.destroy_subscription(self.odom_sub)
            self.destroy_subscription(self.pose_sub)
            self.odom_sub = None
            self.pose_sub = None


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
