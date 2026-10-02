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
    - stop once green covers fill_stop of the frame and the box is centred;
    - while driving forward (far or near), steering w is clamped to
      max_drive_w, not max_w (max_w/min_w are for the turn-in-place case).
festa_demo (2026-10-01, R21 wobble): turning in place is a pulse (about 80 % of
the angle, at most turn_pulse_max s) followed by a stop until an image newer
than stop + settle_s arrives, so camera/detection delay cannot make it
overshoot back and forth. Once the bbox touches a left/right image border
(image_target z != 0) its centre is no longer the box centre: a box wider than
the view (z = 3) counts as centred (creep straight, stop on fill_stop); with one
border cut, lean toward that side while creeping and, once filled, turn toward
it in pulses until both borders are cut. Feedback goes out only when the
state changes (the 20 Hz feedback starved the BT's accept/result replies).
No range estimate or TF. A forward lidar cone (backstop_dist) still stops the
robot. Contact (festa_demo, 2026-10-02, sim S24_R4): the 12 cm box is below the lidar,
and at a diagonal approach the fill stayed at 79-80 % while the robot pushed the box
0.71 m into a wall. So while driving near the box, if odometry says the base moved
contact_travel m but the fill grew less than contact_fill_gain, the robot is touching
(or pushing) the box: from then on it counts as filled - stop when centred or both
borders cut, else turn in pulses toward the cut side as at fill_stop.

Inputs from detector_node.py:
    green_box/image_target  x = (bbox centre column - cx) / fx  (tan, + = right),
                            z = clip: 0 none, 1 left, 2 right, 3 both borders
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from festa_demo.action import Sweep


class VisualApproach(Node):

    def __init__(self):
        super().__init__('visual_approach')
        p = {n: self.declare_parameter(n, d).value for n, d in (
            ('fill_stop', 0.90), ('align_tol', 0.087), ('kp', 1.5), ('max_w', 0.4), ('min_w', 0.12),
            ('settle_s', 0.3), ('turn_pulse_max', 0.5),
            ('max_drive_w', 0.25), ('approach_speed', 0.05), ('far_speed', 0.10), ('near_fill', 0.15),
            ('far_align_tol', 0.26), ('image_timeout', 1.0), ('lost_timeout', 3.0),
            ('backstop_dist', 0.10), ('front_half_angle', 0.26), ('time_allowance', 60.0),
            ('contact_travel', 0.05), ('contact_fill_gain', 0.02))}
        self.p = p
        cb = ReentrantCallbackGroup()
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lock = threading.Lock()
        self.target = None          # (time, x_tan, clip)
        self.fill = None            # (time, fraction)
        self.front_min = None
        self.odom_dist = 0.0        # path length driven (odometry), for the contact check
        self._odom_xy = None
        self._goal_gen = 0  # festa_demo: preemption, 2026-10-01 - only the newest goal may drive
        self._cone_cache = None  # (angle_min, angle_increment, n, indices) - festa_demo: Pi load, 2026-10-01
        self.create_subscription(PointStamped, 'green_box/image_target', self._on_target,
                                 best_effort, callback_group=cb)
        self.create_subscription(Float32, 'green_box/image_fill', self._on_fill,
                                 best_effort, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', self._on_scan, best_effort, callback_group=cb)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=cb)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ActionServer(self, Sweep, 'visual_approach', execute_callback=self._execute,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info('visual_approach ready')

    def _on_target(self, m):
        with self.lock:
            self.target = (time.monotonic(), m.point.x, int(round(m.point.z)))

    def _on_fill(self, m):
        with self.lock:
            self.fill = (time.monotonic(), m.data)

    def _on_odom(self, m):
        xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        with self.lock:
            if self._odom_xy is not None:
                self.odom_dist += math.hypot(xy[0] - self._odom_xy[0], xy[1] - self._odom_xy[1])
            self._odom_xy = xy

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

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    @staticmethod
    def _decide(p, bearing, clip, fill_now, contact=False):
        """festa_demo (2026-10-01): one control decision -> (action, v, w, state);
        action is 'stop', 'turn' (pulse in place) or 'drive'. contact: the robot is
        touching the box (fill stopped growing while driving) - counts as filled."""
        filled = fill_now >= p['fill_stop'] or contact
        if clip == 3:                                  # wider than the view: centred enough
            if filled:
                return 'stop', 0.0, 0.0, 'STOP clip=3'
            return 'drive', p['approach_speed'], 0.0, f'CREEP clip=3 fill={100 * fill_now:.0f}%'
        if clip:
            # One border cut: the box centre lies further toward that border than the
            # visible bbox says (S1e: stopped at -24 deg with only the right edge cut).
            # Lean toward the cut side; once filled, turn there in pulses until both
            # edges are cut or the whole box is back in view.
            side = 1.0 if clip == 1 else -1.0          # + = turn left
            if filled:
                return 'turn', 0.0, side * p['min_w'], f'CENTRE clip={clip}'
            return 'drive', p['approach_speed'], side * 0.5 * p['max_drive_w'], \
                f'CREEP clip={clip} fill={100 * fill_now:.0f}%'
        near = fill_now >= p['near_fill']
        if abs(bearing) > (p['align_tol'] if near else p['far_align_tol']):
            w = max(-p['max_w'], min(p['max_w'], p['kp'] * bearing))
            return 'turn', 0.0, math.copysign(max(abs(w), p['min_w']), w), \
                f'CENTRE bearing={math.degrees(bearing):.0f}deg'
        if filled:
            return 'stop', 0.0, 0.0, 'STOP centred'
        w = max(-p['max_drive_w'], min(p['max_drive_w'], p['kp'] * bearing))
        return 'drive', p['approach_speed'] if near else p['far_speed'], w, \
            f'APPROACH{"" if near else "-FAR"} fill={100 * fill_now:.0f}%'

    def _execute(self, goal):
        p = self.p
        fb = Sweep.Feedback()
        result = Sweep.Result()
        start = time.monotonic()
        last_seen = start
        code = 'ERROR'
        turn_until = 0.0            # festa_demo: end of the current turn pulse
        look_after = None           # festa_demo: wait for an image received after this
        contact = False             # festa_demo (S24_R4): touching the box
        ref = None                  # (odom_dist, fill, cmd_travel) at the start of the current forward drive
        cmd_travel = 0.0            # festa_demo (S25_V4): distance commanded forward (ROS clock)
        last_drive = None           # (ROS time s, v) of the previous loop's forward command
        with self.lock:
            self._goal_gen += 1
            my = self._goal_gen
        while rclpy.ok():
            time.sleep(0.05)
            now = time.monotonic()
            with self.lock:
                preempted = self._goal_gen != my
            if preempted:
                # festa_demo: preemption, 2026-10-01 - a newer goal owns the
                # base now; do not touch cmd_vel, just drop out.
                self.get_logger().warn('visual_approach: preempted by a newer goal')
                goal.abort()
                result.result_code = 'ERROR'
                return result
            if goal.is_cancel_requested:
                self._cmd(0, 0)
                goal.canceled()
                result.result_code = 'ERROR'
                return result
            if now - start > p['time_allowance']:
                self.get_logger().error('visual_approach: time allowance exceeded')
                break
            with self.lock:
                target, fill, front_min, odom_dist = self.target, self.fill, self.front_min, self.odom_dist
            if front_min is not None and front_min <= p['backstop_dist']:
                self.get_logger().info(f'visual_approach: lidar backstop at {front_min:.2f} m')
                code = 'SUCCESS'
                break
            if target is None or now - target[0] > p['image_timeout']:
                self._cmd(0, 0)
                last_drive = None
                if now - last_seen > p['lost_timeout']:
                    self.get_logger().error('visual_approach: box not in view')
                    break
                continue
            last_seen = now
            if now < turn_until:
                last_drive = None
                continue                               # keep the pulse's turn command
            if turn_until:
                self._cmd(0, 0)                        # pulse over: stop, then look
                look_after = now + p['settle_s']
                turn_until = 0.0
            if look_after is not None:
                if target[0] < look_after:
                    last_drive = None
                    continue
                look_after = None
            bearing = -math.atan(target[1])            # + = box left of the image centre
            fill_now = fill[1] if fill is not None and now - fill[0] <= p['image_timeout'] else 0.0
            action, v, w, state = self._decide(p, bearing, target[2], fill_now, contact)
            # festa_demo (2026-10-02, sim S25_V4): travel = max(odometry, commanded). Wedged with
            # the wheels stalled, odometry stood still too and the check never fired (50 sim s).
            t_ros = self.get_clock().now().nanoseconds * 1e-9
            if action == 'drive' and last_drive is not None:
                cmd_travel += last_drive[1] * max(0.0, t_ros - last_drive[0])
            if action == 'drive' and fill_now >= p['near_fill'] and not contact:
                if ref is None:
                    ref = (odom_dist, fill_now, cmd_travel)
                else:
                    travel = max(odom_dist - ref[0], cmd_travel - ref[2])
                    if travel >= p['contact_travel']:
                        if fill_now - ref[1] < p['contact_fill_gain']:
                            contact = True
                            self.get_logger().warn(
                                f'visual_approach: drove {travel:.2f} m (odometry {odom_dist - ref[0]:.2f} m) '
                                f'but the fill only went {100 * ref[1]:.0f}% -> {100 * fill_now:.0f}% '
                                f'- touching the box, counts as filled')
                            action, v, w, state = self._decide(p, bearing, target[2], fill_now, contact)
                        else:
                            ref = (odom_dist, fill_now, cmd_travel)
            elif action != 'drive':
                ref = None
            last_drive = (t_ros, v) if action == 'drive' else None
            if action == 'stop':
                self.get_logger().info(
                    f'visual_approach: box fills {100 * fill_now:.0f}% of the frame, '
                    f'bearing {math.degrees(bearing):.1f} deg, clip {target[2]}'
                    f'{" (contact)" if contact else ""} - stopped')
                code = 'SUCCESS'
                break
            self._cmd(v, w)
            if action == 'turn':
                turn_until = now + (min(p['turn_pulse_max'], 0.8 * abs(bearing) / abs(w))
                                    if not target[2] else p['turn_pulse_max'])
            if state != fb.state:
                fb.state = state
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
