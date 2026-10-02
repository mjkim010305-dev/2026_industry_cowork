#!/usr/bin/env python3
"""/handle_box: clear the green box in one call - camera approach, arm sweep, body push,
escape from a wall - and return one result ("SUCCESS" / "ERROR", festa_demo/action/Sweep).

festa_demo (2026-10-02, user): one program instead of four action servers called one by
one from the BT (/visual_approach, /sweep, /push_through, /escape). Most of the day's
failures came from that split: lost accept replies between nodes (sim S25_V3 arm goal),
a node dying while it created and destroyed subscriptions per goal (real R22
push_through), a new obstacle_clear_sequence.py process (new DDS participant) per sweep,
and Pi load. Here every subscription is made once, the sweep sequence node is made once
and run in this process, and the BT calls /handle_box once per box.

Steps (the logic of the replaced nodes, unchanged unless noted):
  1. approach - visual_approach.py: steer by the bbox, slow down from near_fill, stop at
     fill_stop or on contact (fill not growing while driving: odometry or commanded
     travel); lidar backstop; time_allowance.
  2. wait settle_before_sweep s, then sweep - obstacle_clear_sequence.py's run_all()
     (sequence "obstacle_clear_sequence": rear place -> sweep -> re-pick) or
     sweep_only.py's version (sequence "sweep_only"), in-process.
  3. push - push_through.py: straight distance m; stops on the lidar front cone, on
     /map walls (clearance or the point look_ahead m ahead), on no progress.
  3b. back off - back_off m straight back at back_speed (real R22: Nav2 then turned with the
     box still at the bumper, the wheels slipped and odometry counted 173 deg for a 46 deg
     turn; AMCL followed it ~1 m / 96 deg off). Back along the line the push just drove.
  4. escape - if the centre is within map_stop of a /map wall, drive straight to the side
     with more clearance without a costmap check (Nav2 rejects every motion inside its
     footprint). Also served alone as /escape for the BT's recoveries.
A failed approach or sweep fails the call (the BT retries / gives up); push and escape
are best effort. Robot pose in the map: green_box/map_pose from box_on_path.py.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from festa_demo.action import Sweep

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visual_approach import VisualApproach  # noqa: E402  (_decide only)


class Cancelled(Exception):
    pass


class HandleBox(Node):

    def __init__(self):
        super().__init__('handle_box')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            # approach (visual_approach.py)
            ('fill_stop', 0.90), ('align_tol', 0.087), ('kp', 1.5), ('max_w', 0.4), ('min_w', 0.12),
            ('settle_s', 0.3), ('turn_pulse_max', 0.5),
            ('max_drive_w', 0.25), ('approach_speed', 0.05), ('far_speed', 0.10), ('near_fill', 0.15),
            ('far_align_tol', 0.26), ('image_timeout', 1.0), ('lost_timeout', 3.0),
            ('backstop_dist', 0.10), ('front_half_angle', 0.26), ('approach_allowance', 60.0),
            ('contact_travel', 0.05), ('contact_fill_gain', 0.02),
            # sweep
            ('sequence', 'obstacle_clear_sequence'), ('safety_monitor', True), ('settle_before_sweep', 2.0),
            # push (push_through.py)
            # wall_stop 0: the upright box is in the lidar plane and would stop the push at once;
            # the push stops on /map walls instead (map_stop, map_stop_ahead)
            ('distance', 0.35), ('speed', 0.08), ('wall_stop', 0.0), ('push_allowance', 15.0),
            ('stall_s', 3.0), ('stall_dist', 0.02),
            ('map_stop', 0.21), ('escape_target', 0.25), ('look_ahead', 0.15), ('map_stop_ahead', 0.20),
            # escape
            ('escape_speed', 0.05), ('escape_max', 0.25), ('escape_front', 0.14),
            ('escape_rear', 0.22), ('escape_edge_clear', 0.08), ('pose_timeout', 1.0),
            ('back_off', 0.12), ('back_speed', 0.05))}
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.target = None          # (monotonic time, x_tan, clip)
        self.fill = None            # (monotonic time, fraction)
        self.front_min = None
        self.odom_xy = None
        self.odom_dist = 0.0
        self.dist = None            # (distance field m, origin x, origin y, resolution)
        self.map_pose = None        # (monotonic time, x, y, yaw)
        self._cone_cache = None
        self._goal_gen = 0          # only the newest goal may drive
        self.seq = None             # obstacle_clear_sequence node, made on the first sweep
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        # every subscription once, for the life of the node (real R22: creating and
        # destroying them per goal killed push_through.py)
        self.create_subscription(PointStamped, 'green_box/image_target', self._on_target, be, callback_group=self.cb)
        self.create_subscription(Float32, 'green_box/image_fill', self._on_fill, be, callback_group=self.cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan, be, callback_group=self.cb)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=self.cb)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched, callback_group=self.cb)
        self.create_subscription(PoseStamped, 'green_box/map_pose', self._on_pose, 10, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, Sweep, 'handle_box', execute_callback=self._handle,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.cb)
        ActionServer(self, Sweep, 'escape', execute_callback=self._escape_only,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.cb)
        self.get_logger().info(f'handle_box ready (sequence {self.p["sequence"]}, '
                               f'safety_monitor {self.p["safety_monitor"]})')

    # ---------------------------------------------------------------- inputs
    def _on_target(self, m):
        with self.lock:
            self.target = (time.monotonic(), m.point.x, int(round(m.point.z)))

    def _on_fill(self, m):
        with self.lock:
            self.fill = (time.monotonic(), m.data)

    def _on_scan(self, m):
        key = (m.angle_min, m.angle_increment, len(m.ranges))
        if self._cone_cache is None or self._cone_cache[0] != key:
            half = self.p['front_half_angle']
            self._cone_cache = (key, [i for i in range(len(m.ranges))
                                      if abs(math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                                                        math.cos(m.angle_min + i * m.angle_increment))) <= half])
        best = None
        for i in self._cone_cache[1]:
            r = m.ranges[i]
            if math.isfinite(r) and r > 0.0 and m.range_min <= r <= m.range_max:
                best = r if best is None else min(best, r)
        with self.lock:
            self.front_min = best

    def _on_odom(self, m):
        xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        with self.lock:
            if self.odom_xy is not None:
                self.odom_dist += math.hypot(xy[0] - self.odom_xy[0], xy[1] - self.odom_xy[1])
            self.odom_xy = xy

    def _on_map(self, m):
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

    # ---------------------------------------------------------------- helpers
    def _cmd(self, v, w=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def _clear(self, pose, ahead=0.0):
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

    def _check(self, goal, my):
        """Raise Cancelled when the goal is cancelled or a newer goal took over."""
        with self.lock:
            preempted = self._goal_gen != my
        if preempted:
            raise Cancelled('preempted by a newer goal')
        if goal.is_cancel_requested:
            self._cmd(0)
            raise Cancelled('cancelled')

    def _feedback(self, goal, fb, state):
        if state != fb.state:
            fb.state = state
            goal.publish_feedback(fb)

    # ---------------------------------------------------------------- 1. approach
    def _approach(self, goal, my, fb):
        p = self.p
        start = time.monotonic()
        last_seen = start
        turn_until, look_after = 0.0, None
        contact, ref, cmd_travel, last_drive = False, None, 0.0, None
        while rclpy.ok():
            time.sleep(0.05)
            self._check(goal, my)
            now = time.monotonic()
            if now - start > p['approach_allowance']:
                self.get_logger().error('approach: time allowance exceeded')
                return False
            with self.lock:
                target, fill, front_min, odom_dist = self.target, self.fill, self.front_min, self.odom_dist
            if front_min is not None and front_min <= p['backstop_dist']:
                self.get_logger().info(f'approach: lidar backstop at {front_min:.2f} m')
                return True
            if target is None or now - target[0] > p['image_timeout']:
                self._cmd(0)
                last_drive = None
                if now - last_seen > p['lost_timeout']:
                    self.get_logger().error('approach: box not in view')
                    return False
                continue
            last_seen = now
            if now < turn_until:
                last_drive = None
                continue
            if turn_until:
                self._cmd(0)
                look_after = now + p['settle_s']
                turn_until = 0.0
            if look_after is not None:
                if target[0] < look_after:
                    last_drive = None
                    continue
                look_after = None
            bearing = -math.atan(target[1])
            fill_now = fill[1] if fill is not None and now - fill[0] <= p['image_timeout'] else 0.0
            action, v, w, state = VisualApproach._decide(p, bearing, target[2], fill_now, contact)
            t_ros = self.get_clock().now().nanoseconds * 1e-9
            if action == 'drive' and last_drive is not None:
                cmd_travel += last_drive[1] * max(0.0, t_ros - last_drive[0])
            # contact check only on an unclipped box (real R23: a box cut at the image edge slides
            # out of view while driving, its fill shrank 20 -> 17 % and read as contact)
            if action == 'drive' and fill_now >= p['near_fill'] and not contact and target[2] == 0:
                if ref is None:
                    ref = (odom_dist, fill_now, cmd_travel)
                else:
                    travel = max(odom_dist - ref[0], cmd_travel - ref[2])
                    if travel >= p['contact_travel']:
                        if fill_now - ref[1] < p['contact_fill_gain']:
                            contact = True
                            self.get_logger().warn(
                                f'approach: drove {travel:.2f} m but the fill only went '
                                f'{100 * ref[1]:.0f}% -> {100 * fill_now:.0f}% - touching the box')
                            action, v, w, state = VisualApproach._decide(p, bearing, target[2], fill_now, contact)
                        else:
                            ref = (odom_dist, fill_now, cmd_travel)
            elif action != 'drive':
                ref = None
            last_drive = (t_ros, v) if action == 'drive' else None
            if action == 'stop':
                self.get_logger().info(
                    f'approach: box fills {100 * fill_now:.0f}% of the frame, bearing '
                    f'{math.degrees(bearing):.1f} deg, clip {target[2]}{" (contact)" if contact else ""} - stopped')
                return True
            self._cmd(v, w)
            if action == 'turn':
                turn_until = now + (min(p['turn_pulse_max'], 0.8 * abs(bearing) / abs(w))
                                    if not target[2] else p['turn_pulse_max'])
            self._feedback(goal, fb, state)
        return False

    # ---------------------------------------------------------------- 2. sweep
    def _sequence_node(self):
        if self.seq is None:
            import obstacle_clear_sequence as ocs
            cls = ocs.ObstacleClearSequence
            if self.p['sequence'] == 'sweep_only':
                import sweep_only
                cls = sweep_only.make_class(ocs)
            # a separate node, not in this executor: the sequence spins itself
            # (spin_until_future_complete / spin_once), as when it ran as its own process
            self.seq = cls(safety_enabled=bool(self.p['safety_monitor']))
            if not self.seq.wait_servers():
                self.get_logger().error('sweep: arm/gripper action servers not found')
                self.seq.destroy_node()
                self.seq = None
        return self.seq

    def _sweep(self):
        seq = self._sequence_node()
        if seq is None:
            return False
        ok = bool(seq.run_all())
        self.get_logger().info(f'sweep: {"done" if ok else "FAILED"}')
        return ok

    # ---------------------------------------------------------------- 3. push
    def _push(self, goal, my):
        p = self.p
        start = time.monotonic()
        with self.lock:
            odom0 = self.odom_xy
        progress = (start, 0.0)
        try:
            while rclpy.ok():
                time.sleep(0.05)
                self._check(goal, my)
                if time.monotonic() - start > p['push_allowance']:
                    self.get_logger().error('push: time allowance exceeded')
                    return
                with self.lock:
                    odom_xy, front_min = self.odom_xy, self.front_min
                if odom_xy is None:
                    continue
                if odom0 is None:
                    odom0 = odom_xy
                done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
                if front_min is not None and front_min <= p['wall_stop']:
                    self.get_logger().info(f'push: wall at {front_min:.2f} m after {done:.2f} m - stopped')
                    return
                pose = self._fresh_pose()
                if pose is not None and (self._clear(pose) < p['map_stop']
                                         or self._clear(pose, p['look_ahead']) < p['map_stop_ahead']):
                    self.get_logger().info(
                        f'push: map wall {self._clear(pose):.2f} m (ahead {self._clear(pose, p["look_ahead"]):.2f} m) '
                        f'after {done:.2f} m - stopped')
                    return
                if done >= p['distance']:
                    self.get_logger().info(f'push: pushed {done:.2f} m - done')
                    return
                if done - progress[1] >= p['stall_dist']:
                    progress = (time.monotonic(), done)
                elif time.monotonic() - progress[0] > p['stall_s']:
                    self.get_logger().warn(f'push: no progress for {p["stall_s"]:.0f} s after {done:.2f} m - stopped')
                    return
                self._cmd(p['speed'])
        finally:
            self._cmd(0)

    # ---------------------------------------------------------------- 3b. back off
    def _back_off(self, goal, my):
        p = self.p
        start = time.monotonic()
        with self.lock:
            odom0 = self.odom_xy
        progress = (start, 0.0)
        try:
            while rclpy.ok():
                time.sleep(0.05)
                self._check(goal, my)
                with self.lock:
                    odom_xy = self.odom_xy
                if odom_xy is None or odom0 is None:
                    odom0 = odom_xy
                    continue
                done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
                if done >= p['back_off']:
                    self.get_logger().info(f'back off: {done:.2f} m - done')
                    return
                if done - progress[1] >= p['stall_dist']:
                    progress = (time.monotonic(), done)
                elif time.monotonic() - progress[0] > p['stall_s']:
                    self.get_logger().warn(f'back off: no progress after {done:.2f} m')
                    return
                self._cmd(-p['back_speed'])
        finally:
            self._cmd(0)

    # ---------------------------------------------------------------- 4. escape
    def _escape(self, goal, my):
        """True when clear of the walls (or already was)."""
        p = self.p
        start = time.monotonic()
        pose = None
        while rclpy.ok() and pose is None and time.monotonic() - start < 2.0:
            time.sleep(0.05)
            pose = self._fresh_pose()
        if pose is None:
            self.get_logger().warn('escape: no map pose - not moving')
            return False
        if self._clear(pose) >= p['map_stop']:
            return True
        fwd, back = self._clear(pose, p['look_ahead']), self._clear(pose, -p['look_ahead'])
        sign = 1.0 if fwd > back else -1.0
        edge = p['escape_front'] if sign > 0 else -p['escape_rear']
        self.get_logger().warn(f'escape: {self._clear(pose):.2f} m from a wall (ahead {fwd:.2f}, behind {back:.2f}) '
                               f'- driving {"forward" if sign > 0 else "back"}')
        with self.lock:
            odom0 = self.odom_xy
        progress = (time.monotonic(), 0.0)
        try:
            while rclpy.ok():
                time.sleep(0.05)
                self._check(goal, my)
                pose = self._fresh_pose()
                with self.lock:
                    odom_xy = self.odom_xy
                if pose is None or odom_xy is None:
                    self._cmd(0)
                    if time.monotonic() - progress[0] > p['stall_s']:
                        return False
                    continue
                if odom0 is None:
                    odom0 = odom_xy
                done = math.hypot(odom_xy[0] - odom0[0], odom_xy[1] - odom0[1])
                if self._clear(pose) >= p['escape_target']:
                    self.get_logger().info(f'escape: clear ({self._clear(pose):.2f} m) after {done:.2f} m')
                    return True
                if self._clear(pose, edge) < p['escape_edge_clear'] or done >= p['escape_max']:
                    self.get_logger().warn(f'escape: stopped after {done:.2f} m, still {self._clear(pose):.2f} m from a wall')
                    return False
                if done - progress[1] >= p['stall_dist']:
                    progress = (time.monotonic(), done)
                elif time.monotonic() - progress[0] > p['stall_s']:
                    self.get_logger().warn(f'escape: no progress after {done:.2f} m')
                    return False
                self._cmd(sign * p['escape_speed'])
        finally:
            self._cmd(0)

    # ---------------------------------------------------------------- actions
    def _start(self):
        with self.lock:
            self._goal_gen += 1
            return self._goal_gen

    def _finish(self, goal, ok):
        result = Sweep.Result()
        result.result_code = 'SUCCESS' if ok else 'ERROR'
        if ok:
            goal.succeed()
        else:
            goal.abort()
        return result

    def _handle(self, goal):
        my = self._start()
        fb = Sweep.Feedback()
        try:
            self._feedback(goal, fb, 'APPROACH')
            if not self._approach(goal, my, fb):
                self._cmd(0)
                return self._finish(goal, False)
            self._cmd(0)
            self._feedback(goal, fb, 'SWEEP')
            time.sleep(self.p['settle_before_sweep'])
            self._check(goal, my)
            if not self._sweep():
                return self._finish(goal, False)
            self._check(goal, my)
            self._feedback(goal, fb, 'PUSH')
            self._push(goal, my)
            self._feedback(goal, fb, 'BACK_OFF')
            self._back_off(goal, my)
            self._feedback(goal, fb, 'ESCAPE')
            self._escape(goal, my)
            self.get_logger().info('handle_box: done')
            return self._finish(goal, True)
        except Cancelled as e:
            self.get_logger().warn(f'handle_box: {e}')
            if goal.is_cancel_requested:
                goal.canceled()
                return Sweep.Result(result_code='ERROR')
            return self._finish(goal, False)

    def _escape_only(self, goal):
        my = self._start()
        try:
            return self._finish(goal, self._escape(goal, my))
        except Cancelled:
            if goal.is_cancel_requested:
                goal.canceled()
                return Sweep.Result(result_code='ERROR')
            return self._finish(goal, False)


def main():
    rclpy.init()
    node = HandleBox()
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
