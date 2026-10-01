#!/usr/bin/env python3
"""green_box/on_path: is the detected green box on the way to the goal?

The real course (2026-10-01) is 1.45 m wide, so one 18.5 cm box never blocks
it and "sweep only when blocked" would just drive around the box. User
decision: also sweep when the box is on the path. The global plan already
bends around the box (inflation), so "on the path" means the next
`lookahead` metres of `plan` pass within `on_path_dist` of the box centre
(both in the map frame). Published at 5 Hz; bt/festa_demo.xml reads it
through IsGreenBoxDetected(visible_topic="green_box/on_path"), which also
requires a fresh green_box/pose.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool


class BoxOnPath(Node):

    def __init__(self):
        super().__init__('box_on_path')
        self.on_path_dist = float(self.declare_parameter('on_path_dist', 0.6).value)
        self.lookahead = float(self.declare_parameter('lookahead', 2.0).value)
        self.path = None
        self.box = None
        self.last = None
        self.create_subscription(Path, 'plan', self._on_path, 10)
        self.create_subscription(PoseStamped, 'green_box/pose', self._on_box, 10)
        self.pub = self.create_publisher(Bool, 'green_box/on_path', 10)
        self.create_timer(0.2, self._tick)

    def _on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_box(self, msg):
        self.box = (msg.pose.position.x, msg.pose.position.y)

    def _tick(self):
        on = False
        if self.path and self.box:
            travelled, prev = 0.0, self.path[0]
            for x, y in self.path:
                travelled += math.hypot(x - prev[0], y - prev[1])
                prev = (x, y)
                if travelled > self.lookahead:
                    break
                if math.hypot(x - self.box[0], y - self.box[1]) < self.on_path_dist:
                    on = True
                    break
        if on != self.last:
            self.get_logger().info(f'green box on path: {on}')
            self.last = on
        self.pub.publish(Bool(data=on))


def main():
    rclpy.init()
    rclpy.spin(BoxOnPath())


if __name__ == '__main__':
    main()
