#!/usr/bin/env python3
"""green_box/on_path: is the green box right in front of the robot, close enough to clear?

Image-only (user, 2026-10-02: "a small bbox -> keep going; once the box takes up a lot
of the frame -> handle it"; dynamic scenes, so every box is judged afresh, on the way
out and on the way back, also one that appears right after a corner). Replaces the
map-based check (box pose within 0.4 m of the next 2 m of /plan), which depended on
AMCL, the camera TF and the image-width range and re-triggered on a box already swept
aside (sim S1l).

From detector_node.py's green_box/bbox ([x, y, w, h, image_w, image_h, clip, fill],
pixels; clip: 0 none, 1 left, 2 right, 3 both borders) and the camera intrinsics:
    distance d  = H * fy / h  while the bbox touches neither the top nor the bottom
                  border (the height does not grow when the box is turned, the width
                  does: two faces show), else W * fx / w   (H = box_height_m,
                  W = box_face_width_m, the calibrated effective face width)
    sideways  s = (u_centre - cx) / fx * d - camera_y   (metres from the centre line)
    median of the last median_n measurements within image_timeout s (bbox jitter)
on  when d <= trigger_dist and |s| <= lateral_tol      (robot and box touch within
    ~0.24 m sideways: half widths 0.15 + 0.09; 0.30 leaves a little margin);
off when d > release_dist or |s| > release_lateral, or no image for image_timeout s.
A bbox cut by both borders (z = 3) is wider than the view: right in front. A bbox cut
by one border has an unreliable centre and width, but its inner edge is real: the box
reaches into the robot's way when that edge is within lateral_tol - box_half_m of the
centre line (on; off beyond release_lateral - box_half_m). Distance from the bbox height
when it touches neither top nor bottom, else the box is close (clip_close_dist).
(festa_demo 2026-10-02, sim S24_R4: "keep the previous decision" latched on for a box
pushed against a wall right after it was handled, and the robot went for it again.)
Not while turning (|odom angular z| > max_turn_rate in the last turn_hold s): mid-corner
the robot faces a different way than it will drive (sim S1m: on the way back it aimed
at a box swept against the wall, 0.44 m off the route); judge again once straight.
Except for a box nearer than turn_gate_dist: sim R2 curved through a corner straight
into a box just past it, still turning, and never triggered.

No map, AMCL or camera TF involved. Published at 5 Hz: green_box/on_path (Bool) and,
while on, green_box/front (PoseStamped in base_link: x = d, y = s), which
bt/festa_demo.xml's IsGreenBoxDetected uses for freshness.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32MultiArray


class BoxOnPath(Node):

    def __init__(self):
        super().__init__('box_on_path')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            ('box_face_width_m', 0.24), ('box_height_m', 0.12), ('camera_info_topic', 'camera_info'),
            ('edge_margin_px', 3), ('median_n', 5), ('camera_y', -0.0115), ('trigger_dist', 1.0),
            ('lateral_tol', 0.30), ('release_dist', 1.2), ('release_lateral', 0.35),
            ('image_timeout', 1.0), ('max_turn_rate', 0.2), ('turn_hold', 0.5),
            ('turn_gate_dist', 0.6), ('box_half_m', 0.09), ('clip_close_dist', 0.3))}
        self.target = None          # (monotonic time, d, s, clip) of the newest bbox
                                    # (clip 1/2: s = lateral position of the inner edge)
        self.recent = []            # (monotonic time, d, s) of unclipped bboxes, for the median
        self.intrinsics = None      # (fx, fy, cx, cy)
        self.on = False
        self.front = (0.0, 0.0)
        self.last_turn = 0.0        # monotonic time the base last turned faster than max_turn_rate
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32MultiArray, 'green_box/bbox', self._on_bbox, best_effort)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info, best_effort)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.pub = self.create_publisher(Bool, 'green_box/on_path', 10)
        self.front_pub = self.create_publisher(PoseStamped, 'green_box/front', 10)
        self.create_timer(0.2, self._tick)

    def _on_info(self, m):
        if m.k[0] > 0.0:
            self.intrinsics = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_bbox(self, m):
        if self.intrinsics is None or len(m.data) < 8 or m.data[2] <= 0.0:
            return
        x, y, w, h, _, img_h, clip, _ = m.data[:8]
        fx, fy, cx, _ = self.intrinsics
        p, now = self.p, time.monotonic()
        m_px = p['edge_margin_px']
        clip = int(round(clip))
        vertical_ok = y > m_px and y + h < img_h - m_px and h > 0.0
        if vertical_ok:
            d = p['box_height_m'] * fy / h
        elif clip:
            d = p['clip_close_dist']        # cut at a side and at top/bottom: right at the robot
        else:
            d = p['box_face_width_m'] * fx / w
        u = x + w / 2.0 if clip == 0 else (x + w if clip == 1 else x)   # clip 1: right edge, 2: left edge
        s = (u - cx) / fx * d - p['camera_y']
        self.target = (now, d, s, clip)
        if clip == 0:
            self.recent = [r for r in self.recent if now - r[0] <= p['image_timeout']][-(p['median_n'] - 1):]
            self.recent.append((now, d, s))

    def _on_odom(self, m):
        if abs(m.twist.twist.angular.z) > self.p['max_turn_rate']:
            self.last_turn = time.monotonic()

    def _judge(self, d, off, on_tol, off_tol):
        p = self.p
        turning = (time.monotonic() - self.last_turn < p['turn_hold']
                   and d > p['turn_gate_dist'])
        if d <= p['trigger_dist'] and off <= on_tol and not turning:
            self.on = True
        elif d > p['release_dist'] or off > off_tol:
            self.on = False

    def _tick(self):
        p, t = self.p, self.target
        was = self.on
        if t is None or time.monotonic() - t[0] > p['image_timeout']:
            self.on = False
        elif t[3] == 3:
            self.on, self.front = True, (0.0, 0.0)
        elif t[3] == 0 and self.recent:
            ds = sorted(r[1] for r in self.recent)
            ss = sorted(r[2] for r in self.recent)
            d, s = ds[len(ds) // 2], ss[len(ss) // 2]
            self._judge(d, abs(s), p['lateral_tol'], p['release_lateral'])
            self.front = (d, s)
        elif t[3] in (1, 2):
            # inner edge s (right-positive): clip 1 = box to the left of s, clip 2 = to the right
            d, s = t[1], t[2]
            reach = -s if t[3] == 1 else s     # how far the box stays away from the centre line
            self._judge(d, max(reach, 0.0), p['lateral_tol'] - p['box_half_m'],
                        p['release_lateral'] - p['box_half_m'])
            self.front = (d, s)
        if self.on != was:
            self.get_logger().info(
                f'green box in front: {self.on} (d={self.front[0]:.2f} m, side={self.front[1]:+.2f} m)')
        self.pub.publish(Bool(data=self.on))
        if self.on:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x, pose.pose.position.y = self.front
            pose.pose.orientation.w = 1.0
            self.front_pub.publish(pose)


def main():
    rclpy.init()
    rclpy.spin(BoxOnPath())


if __name__ == '__main__':
    main()
