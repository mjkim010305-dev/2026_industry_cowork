#!/usr/bin/env python3
"""green_box/on_path: is the green box right in front of the robot, close enough to clear?

Image-only (user, 2026-10-02: "a small bbox -> keep going; once the box takes up a lot
of the frame -> handle it"; dynamic scenes, so every box is judged afresh, on the way
out and on the way back, also one that appears right after a corner). Replaces the
map-based check (box pose within 0.4 m of the next 2 m of /plan), which depended on
AMCL, the camera TF and the image-width range and re-triggered on a box already swept
aside (sim S1l).

From detector_node.py's green_box/image_target (x = bbox centre offset / fx, y = bbox
width / fx, z = clip: 0 none, 1 left, 2 right, 3 both borders) and the known box face
width W (box_face_width_m):
    distance d  = W / y                    (the box face spans y = W / d)
    sideways  s = x * d - camera_y         (metres from the robot's centre line)
on  when d <= trigger_dist and |s| <= lateral_tol      (robot and box touch within
    ~0.24 m sideways: half widths 0.15 + 0.09; 0.30 leaves a little margin);
off when d > release_dist or |s| > release_lateral, or no image for image_timeout s.
A bbox cut by both borders (z = 3) is wider than the view: right in front. A bbox cut
by one border has an unreliable centre and width: keep the previous decision.

No map, AMCL or camera TF involved. Published at 5 Hz: green_box/on_path (Bool) and,
while on, green_box/front (PoseStamped in base_link: x = d, y = s), which
bt/festa_demo.xml's IsGreenBoxDetected uses for freshness.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool


class BoxOnPath(Node):

    def __init__(self):
        super().__init__('box_on_path')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            ('box_face_width_m', 0.24), ('camera_y', -0.0115), ('trigger_dist', 1.0),
            ('lateral_tol', 0.30), ('release_dist', 1.2), ('release_lateral', 0.35),
            ('image_timeout', 1.0))}
        self.target = None          # (monotonic time, x_tan, w_tan, clip)
        self.on = False
        self.front = (0.0, 0.0)
        self.create_subscription(PointStamped, 'green_box/image_target', self._on_target,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(Bool, 'green_box/on_path', 10)
        self.front_pub = self.create_publisher(PoseStamped, 'green_box/front', 10)
        self.create_timer(0.2, self._tick)

    def _on_target(self, m):
        self.target = (time.monotonic(), m.point.x, m.point.y, int(round(m.point.z)))

    def _tick(self):
        p, t = self.p, self.target
        was = self.on
        if t is None or time.monotonic() - t[0] > p['image_timeout']:
            self.on = False
        elif t[3] == 3:
            self.on, self.front = True, (0.0, 0.0)
        elif t[3] == 0 and t[2] > 0.0:
            d = p['box_face_width_m'] / t[2]
            s = t[1] * d - p['camera_y']
            if d <= p['trigger_dist'] and abs(s) <= p['lateral_tol']:
                self.on = True
            elif d > p['release_dist'] or abs(s) > p['release_lateral']:
                self.on = False
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
