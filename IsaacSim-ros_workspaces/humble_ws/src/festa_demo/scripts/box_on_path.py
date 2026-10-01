#!/usr/bin/env python3
"""green_box/on_path: is the green box in the robot's way, close enough to clear?

The box is measured from the camera image only (the 12 cm box is under the lidar);
"in the way" is judged against the route Nav2 is about to drive (/plan), not the
camera axis.

History (festa_demo): user 2026-10-02 "a small bbox -> keep going; once the box takes up
a lot of the frame -> handle it", dynamic scenes, every box judged afresh. The first image
trigger judged against the camera axis; that went wrong three times in sim: S24_R4 re-took
a box pinned against a wall next to the robot right after handling it, S25_V1 took a box
0.375 m off the route mid-turn and jammed it into a pillar corner, S25_V3 cycled 10+ times
on a box between its nose and a wall in the goal pocket while the route back led away.
The camera points wherever a stall, approach or push left it.

Box position in base_link, from detector_node.py's green_box/bbox ([x, y, w, h, image_w,
image_h, clip, fill], pixels; clip: 0 none, 1 left, 2 right, 3 both borders) and the
camera intrinsics:
    distance d  = H * fy / h while the bbox touches neither the top nor the bottom border
                  (the height does not grow when the box is turned), else W * fx / w
                  (cut at a side as well: clip_close_dist)
    box x = camera_x + d,  box y = camera_y - (u - cx) / fx * d
    clip 0: u = bbox centre, median of the last median_n frames within image_timeout s
    clip 1/2: u = the visible inner edge, box centre box_centre_off m beyond it
    clip 3 (wider than the view): straight ahead at clip_close_dist
Route in base_link: /plan (map frame) through map->base_link = map->odom (from /amcl_pose
and the odometry sample at its stamp) * odom->base_link (latest /odom). Only these two
topics, no TF listener (Pi load). AMCL's absolute error mostly cancels: the plan starts at
the AMCL pose.
    along, lat = projection of the box onto the next route_len m of the route
    on   when along_min <= along <= trigger_dist and lat <= lateral_tol, unless the route
         turns away first (route point route_look m ahead more than hold_bearing off the
         nose: Nav2 turns in place before driving, the box ahead is not in the way yet)
    off  when along > release_dist or lat > release_lateral, no image for image_timeout s,
         or no route (plan older than plan_timeout s, no AMCL yet)
Published at 5 Hz: green_box/on_path (Bool) and, while on, green_box/front (PoseStamped,
base_link, the box position), which bt/festa_demo.xml's IsGreenBoxDetected uses for freshness.
"""
import bisect
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32MultiArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """a * b for 2D poses (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def project(pt, route):
    """(along, lat) of pt on the polyline route (list of (x, y), starting at the robot)."""
    best, acc = None, 0.0
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        t = max(0.0, min(1.0, ((pt[0] - ax) * dx + (pt[1] - ay) * dy) / (seg * seg)))
        lat = math.hypot(pt[0] - ax - t * dx, pt[1] - ay - t * dy)
        if best is None or lat < best[1]:
            best = (acc + t * seg, lat)
        acc += seg
    return best if best is not None else (0.0, math.hypot(*pt))


class BoxOnPath(Node):

    def __init__(self):
        super().__init__('box_on_path')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            ('box_face_width_m', 0.24), ('box_height_m', 0.12), ('camera_info_topic', 'camera_info'),
            ('camera_x', 0.1205), ('camera_y', -0.0115),
            ('edge_margin_px', 3), ('median_n', 5), ('image_timeout', 1.0),
            ('clip_close_dist', 0.25), ('box_centre_off', 0.11),
            ('along_min', 0.10), ('trigger_dist', 1.1), ('lateral_tol', 0.30),
            ('release_dist', 1.3), ('release_lateral', 0.40),
            ('route_len', 1.5), ('route_look', 0.4), ('hold_bearing', 0.785),
            ('plan_timeout', 3.0), ('odom_keep', 5.0))}
        self.target = None          # (monotonic time, clip, (x, y) box in base_link)
        self.recent = []            # (monotonic time, x, y) of unclipped bboxes, for the median
        self.intrinsics = None      # (fx, fy, cx, cy)
        self.on = False
        self.front = (0.0, 0.0)     # box (x, y) in base_link
        self.odom_hist = []         # (stamp s, (x, y, yaw)) for map->odom at the AMCL stamp
        self.map_odom = None
        self.plan = None            # (node clock s when received, [(x, y)] in map)
        self.why = ''
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Float32MultiArray, 'green_box/bbox', self._on_bbox, best_effort)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info, best_effort)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.pub = self.create_publisher(Bool, 'green_box/on_path', 10)
        self.front_pub = self.create_publisher(PoseStamped, 'green_box/front', 10)
        self.create_timer(0.2, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, m):
        if m.k[0] > 0.0:
            self.intrinsics = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.odom_hist.append((t, pose))
        while self.odom_hist and t - self.odom_hist[0][0] > self.p['odom_keep']:
            self.odom_hist.pop(0)

    def _on_amcl(self, m):
        if not self.odom_hist:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ts = [h[0] for h in self.odom_hist]
        i = bisect.bisect_left(ts, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(ts)), key=lambda k: abs(ts[k] - t))
        if abs(ts[j] - t) > 0.5:
            return
        amcl = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
        self.map_odom = compose(amcl, inverse(self.odom_hist[j][1]))

    def _on_plan(self, m):
        self.plan = (self._now(), [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses])

    def _on_bbox(self, m):
        if self.intrinsics is None or len(m.data) < 8 or m.data[2] <= 0.0:
            return
        x, y, w, h, _, img_h, clip, _ = m.data[:8]
        fx, fy, cx, _ = self.intrinsics
        p, now = self.p, time.monotonic()
        m_px = p['edge_margin_px']
        clip = int(round(clip))
        if clip == 3:
            self.target = (now, 3, (p['camera_x'] + p['clip_close_dist'], p['camera_y']))
            return
        if y > m_px and y + h < img_h - m_px and h > 0.0:
            d = p['box_height_m'] * fy / h
        elif clip:
            d = p['clip_close_dist']
        else:
            d = p['box_face_width_m'] * fx / w
        u = x + w / 2.0 if clip == 0 else (x + w if clip == 1 else x)
        by = p['camera_y'] - (u - cx) / fx * d
        if clip == 1:                   # cut at the left border: the box reaches further left (+y)
            by += p['box_centre_off']
        elif clip == 2:
            by -= p['box_centre_off']
        bx = p['camera_x'] + d
        self.target = (now, clip, (bx, by))
        if clip == 0:
            self.recent = [r for r in self.recent if now - r[0] <= p['image_timeout']][-(p['median_n'] - 1):]
            self.recent.append((now, bx, by))

    def _route(self):
        """Next route_len m of the plan in base_link, from the point nearest the robot; None if unknown."""
        p = self.p
        if self.plan is None or self.map_odom is None or not self.odom_hist:
            return None
        if self._now() - self.plan[0] > p['plan_timeout'] or len(self.plan[1]) < 2:
            return None
        base = inverse(compose(self.map_odom, self.odom_hist[-1][1]))
        c, s = math.cos(base[2]), math.sin(base[2])
        pts = [(base[0] + c * x - s * y, base[1] + s * x + c * y) for x, y in self.plan[1]]
        i0 = min(range(len(pts)), key=lambda i: math.hypot(*pts[i]))
        route, acc = [pts[i0]], 0.0
        for q in pts[i0 + 1:]:
            acc += math.hypot(q[0] - route[-1][0], q[1] - route[-1][1])
            route.append(q)
            if acc >= p['route_len']:
                break
        return route if len(route) >= 2 else None

    def _tick(self):
        p, t = self.p, self.target
        was = self.on
        route = self._route()
        if t is None or time.monotonic() - t[0] > p['image_timeout']:
            self.on, self.why = False, 'no box in view'
        elif route is None:
            self.on, self.why = False, 'no route'
        else:
            if t[1] == 0 and self.recent:
                xs = sorted(r[1] for r in self.recent)
                ys = sorted(r[2] for r in self.recent)
                box = (xs[len(xs) // 2], ys[len(ys) // 2])
            else:
                box = t[2]
            along, lat = project(box, route)
            look, acc = route[-1], 0.0
            for a, b in zip(route, route[1:]):
                acc += math.hypot(b[0] - a[0], b[1] - a[1])
                if acc >= p['route_look']:
                    look = b
                    break
            hold = abs(math.atan2(look[1], look[0])) > p['hold_bearing']
            if not hold and p['along_min'] <= along <= p['trigger_dist'] and lat <= p['lateral_tol']:
                self.on = True
            elif along > p['release_dist'] or lat > p['release_lateral'] or along < p['along_min'] - 0.05:
                self.on = False
            self.front = box
            self.why = (f'along {along:.2f} m, off route {lat:.2f} m, clip {t[1]}'
                        + (', route turns away' if hold else ''))
        if self.on != was:
            self.get_logger().info(
                f'green box in front: {self.on} (box x={self.front[0]:.2f} y={self.front[1]:+.2f} m; {self.why})')
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
