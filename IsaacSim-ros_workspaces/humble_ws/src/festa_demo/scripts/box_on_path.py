#!/usr/bin/env python3
"""Green boxes seen by the camera -> Nav2's costmaps; green_box/on_path for the BT.

festa_demo (2026-10-02, user, real R24/R25c): the robot went to clear boxes that were only
near its route ("it does not think about the path it will drive"). The box (13.5 cm, fixed
by the event) is under the lidar, so Nav2 planned straight through it. Now:
  - every box the camera sees is remembered in the map frame and drawn into
    green_box/box_map (OccupancyGrid on /map's grid, lethal disc of box_radius m),
    which a StaticLayer ("box_layer") adds to the global and local costmaps: Nav2
    plans around the box when it can;
  - a remembered box is forgotten when its spot is in clear view (forget_min..forget_max
    m ahead, within forget_bearing) and no box is seen near it for forget_s s
    (taken away, or swept elsewhere - then the new spot is remembered);
  - green_box/on_path = a box in front (x <= trigger_dist, |y| <= lateral_tol in base_link);
    the BT clears it only when the planner also failed (goal blocked), so the range is wide.
  (2026-10-02, later: standing on its side - 18.5 cm - the lidar sees the box, but AMCL then
  fitted the unmapped box to walls and the pose jumped (real R26-R29); the box lies flat
  again and box_map is on.)

Box position in base_link from detector_node.py's green_box/bbox ([x, y, w, h, image_w,
image_h, clip, fill]; clip 0 none, 1 left, 2 right, 3 both borders) and the intrinsics
(real calibration 2026-10-02, 19 placements: mean 1.8 cm in distance, 1.0 cm sideways):
    distance d  = H * fy / h while the bbox touches neither top nor bottom (else W * fx / w;
                  cut at a side as well: clip_close_dist)
    box x = camera_x + d,  box y = camera_y - (u - cx) / fx * d
    clip 0: u = bbox centre; more than side_edge_from m to a side, the outer edge minus half
            a face (the inner side face pulls the centre in); median of median_n frames
    clip 1/2: u = the visible inner edge, centre box_centre_off m beyond it
    clip 3: straight ahead at clip_close_dist
Robot pose in the map = map->odom (amcl_pose + the odometry at its stamp) * latest odom; no
TF listener (Pi load). Also published: green_box/front (box in base_link while on) and
green_box/map_pose (robot in the map, for handle_box.py).
"""
import bisect
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
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


class BoxOnPath(Node):

    def __init__(self):
        super().__init__('box_on_path')
        self.p = {n: self.declare_parameter(n, d).value for n, d in (
            ('box_face_width_m', 0.24), ('box_height_m', 0.12), ('camera_info_topic', 'camera_info'),
            ('camera_x', 0.1205), ('camera_y', -0.0115),
            ('edge_margin_px', 3), ('median_n', 5), ('image_timeout', 1.0),
            ('clip_close_dist', 0.25), ('box_centre_off', 0.11),
            ('side_edge_from', 0.12), ('box_half_face', 0.0925),
            ('trigger_dist', 1.8), ('lateral_tol', 0.50), ('release_dist', 2.0), ('release_lateral', 0.60),
            ('box_map', True),
            ('box_radius', 0.13), ('merge_dist', 0.35), ('forget_min', 0.35), ('forget_max', 1.3),
            ('forget_bearing', 0.6), ('forget_s', 1.5), ('odom_keep', 5.0), ('odom_every', 5))}
        self.target = None          # (monotonic time, clip, (x, y) box in base_link)
        self.recent = []            # (monotonic time, x, y) of unclipped bboxes, for the median
        self.intrinsics = None      # (fx, fy, cx, cy)
        self.on = False
        self.front = (0.0, 0.0)     # box (x, y) in base_link
        self.odom_hist = []         # (stamp s, (x, y, yaw)) for map->odom at the AMCL stamp
        self.map_odom = None
        self.boxes = []             # remembered boxes: [x, y, last seen (monotonic), unseen since or None]
        self.grid_info = None       # /map's MapMetaData
        self.box_cells = None       # cells drawn last time (to republish only on change)
        self.why = ''
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Float32MultiArray, 'green_box/bbox', self._on_bbox, best_effort)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info, best_effort)
        # festa_demo (2026-10-02, real R28): /odom comes at ~49 Hz; decode one in odom_every
        # (~10 Hz is plenty for the AMCL-stamp match and the robot pose)
        self._odom_n = 0
        self.create_subscription(Odometry, 'odom', self._on_odom_raw, 10, raw=True)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.pub = self.create_publisher(Bool, 'green_box/on_path', 10)
        self.front_pub = self.create_publisher(PoseStamped, 'green_box/front', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'green_box/map_pose', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'green_box/box_map', latched)
        self.create_timer(0.2, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, m):
        if m.k[0] > 0.0:
            self.intrinsics = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _on_odom_raw(self, data):
        self._odom_n += 1
        if self._odom_n % self.p['odom_every'] == 0:
            self._on_odom(deserialize_message(data, Odometry))

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

    def _on_map(self, m):
        self.grid_info = m.info
        self.box_cells = None       # draw (an empty) box map on the new grid

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
        if clip == 0 and abs(by - p['camera_y']) > p['side_edge_from']:
            # off to one side: outer edge (left edge for a box on the left, +y) minus half a face
            u_out = x if by > p['camera_y'] else x + w
            y_out = p['camera_y'] - (u_out - cx) / fx * d
            by = y_out - math.copysign(p['box_half_face'], y_out - p['camera_y'])
        if clip == 1:                   # cut at the left border: the box reaches further left (+y)
            by += p['box_centre_off']
        elif clip == 2:
            by -= p['box_centre_off']
        bx = p['camera_x'] + d
        self.target = (now, clip, (bx, by))
        if clip == 0:
            self.recent = [r for r in self.recent if now - r[0] <= p['image_timeout']][-(p['median_n'] - 1):]
            self.recent.append((now, bx, by))

    def _publish_map_pose(self):
        if self.map_odom is None or not self.odom_hist:
            return
        x, y, yaw = compose(self.map_odom, self.odom_hist[-1][1])
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        self.pose_pub.publish(m)

    def _robot(self):
        if self.map_odom is None or not self.odom_hist:
            return None
        return compose(self.map_odom, self.odom_hist[-1][1])

    def _remember(self, robot, box, clip, now):
        """Update the box memory with one measurement (box in base_link)."""
        p = self.p
        bx, by, _ = compose(robot, (box[0], box[1], 0.0))
        near = [b for b in self.boxes if math.hypot(b[0] - bx, b[1] - by) < p['merge_dist']]
        if near:
            b = min(near, key=lambda b: math.hypot(b[0] - bx, b[1] - by))
            if clip == 0:           # clipped boxes only refresh, they do not move a box
                b[0], b[1] = 0.7 * b[0] + 0.3 * bx, 0.7 * b[1] + 0.3 * by
            b[2], b[3] = now, None
        elif clip == 0:
            self.boxes.append([bx, by, now, None])
            self.get_logger().info(f'box remembered at ({bx:.2f}, {by:.2f}) - {len(self.boxes)} in the costmap')

    def _forget(self, robot, seen, now):
        """Forget boxes whose spot is in clear view while no box is seen there."""
        p = self.p
        inv = inverse(robot)
        keep = []
        for b in self.boxes:
            x, y, _ = compose(inv, (b[0], b[1], 0.0))
            in_view = p['forget_min'] < x < p['forget_max'] and abs(math.atan2(y, x)) < p['forget_bearing']
            there = seen is not None and math.hypot(seen[0] - x, seen[1] - y) < p['merge_dist']
            if in_view and not there:
                b[3] = b[3] if b[3] is not None else now
                if now - b[3] > p['forget_s']:
                    self.get_logger().info(f'box at ({b[0]:.2f}, {b[1]:.2f}) is gone - removed from the costmap')
                    continue
            else:
                b[3] = None
            keep.append(b)
        self.boxes = keep

    def _publish_box_map(self):
        info = self.grid_info
        if info is None:
            return
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        r = int(math.ceil(self.p['box_radius'] / res))
        cells = set()
        for b in self.boxes:
            ci, cj = int((b[0] - ox) / res), int((b[1] - oy) / res)
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    if (di * di + dj * dj) * res * res <= self.p['box_radius'] ** 2:
                        i, j = ci + di, cj + dj
                        if 0 <= i < info.width and 0 <= j < info.height:
                            cells.add(j * info.width + i)
        if cells == self.box_cells:
            return
        self.box_cells = cells
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.info = info
        data = [0] * (info.width * info.height)
        for k in cells:
            data[k] = 100
        m.data = data
        self.map_pub.publish(m)

    def _tick(self):
        p, t = self.p, self.target
        now = time.monotonic()
        was = self.on
        self._publish_map_pose()
        robot = self._robot()
        box = None
        if t is not None and now - t[0] <= p['image_timeout']:
            if t[1] == 0 and self.recent:
                xs = sorted(r[1] for r in self.recent)
                ys = sorted(r[2] for r in self.recent)
                box = (xs[len(xs) // 2], ys[len(ys) // 2])
            else:
                box = t[2]
        if p['box_map']:
            if robot is not None:
                if box is not None:
                    self._remember(robot, box, t[1], now)
                self._forget(robot, box, now)
            # also before AMCL has a pose: the costmaps' box_layer waits for a first (empty) map
            self._publish_box_map()
        if box is None:
            self.on, self.why = False, 'no box in view'
        else:
            x, y = box
            if x <= p['trigger_dist'] and abs(y) <= p['lateral_tol']:
                self.on = True
            elif x > p['release_dist'] or abs(y) > p['release_lateral']:
                self.on = False
            self.front = box
            self.why = f'clip {t[1]}'
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
