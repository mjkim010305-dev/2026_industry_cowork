#!/usr/bin/env python3
"""Find the obstacle that sits on the planned path and is not on the static map.

The local costmap has no static layer, so walls and racks are 254 there too.
Per cycle (5 Hz):

  1. take the lethal (== 254) cells of /local_costmap/costmap_raw and put
     their centers in the map frame (TF map <- costmap frame)
  2. drop every cell within static_margin of an occupied /map cell
     and every cell inside a /mona/obstacle_marks/list square (already detoured)
  3. gap merge ("a gap the robot can't pass is blocked"): a remaining cell that
     is connected to a mark through gaps <= merge_gap joins that mark; it is not
     a trigger, and those cells (snapped to the grid) go out on
     /mona/obstacle_marks/extend (std_msgs/String JSON) for obstacle_marks
  4. cluster the rest (8-connected)
  5. look only at the path from the robot to trigger_dist ahead; a cluster is
     "on the path" if one of its cells is within corridor_half of that
     polyline (not before its start, not beyond its end); the one with the
     smallest along-path distance wins
  6. fit the visible front face of the winner (PCA of the cell centers)

Sweep tracking (/mona/swept_objects/add, cleared when a goal ends): every swept
object keeps its original face mid, approach heading, the predicted spot after
the sweep (mid shifted to the robot's left) and a sweep count. A cluster whose
centroid is near an entry is that object; if it now lies off the original
approach line by more than corridor_half it was pushed out of the way and is
skipped in step 5. Otherwise it can trigger again and the published
PathObstacle carries sweep_count (0 = not a tracked object).
Pushed objects (/mona/pushed_objects/add, sent right after a push ends) are tracked the
same way with kind "push" and published as push_count: pred = robot + heading *
push_tip_offset; a cluster belongs to the entry if it lies within corridor_half of the
approach line and -0.3 .. push_assoc_along m along it. Push entries are never skipped.

and publishes mona_interfaces/PathObstacle on /mona/path_obstacle, all
geometry in the map frame. Only the face the lidar saw is observed; the depth
behind it is unknown (see the .msg). Marker for RViz: /mona/path_obstacle/marker.

Ignore list (/mona/ignored_objects/add, param ignore_ttl, cleared when a goal ends): a
cluster whose centroid lies within an entry's radius is skipped like a pushed-out object.
The BT adds an obstacle there when the judge twice found no candidate (phantom / out of FOV).

Pure functions come first so they test without rclpy (test_path_obstacle_monitor.py).
"""
import json
import math
import time

import numpy as np

MAX_FACE_POINTS = 32
ROUND_EIGEN_RATIO = 2.0    # PCA eigenvalue ratio below this = "round", no usable line
OCCUPIED_THRESH = 65       # /map cells >= this are static obstacles


def costmap_cells_to_map(data, size_x, size_y, resolution, origin_xy, origin_yaw,
                         lethal, tf_map_costmap):
    """Lethal cells of a row-major costmap -> (ij Nx2 int [x, y], xy Nx2 map-frame
    centers). tf_map_costmap = (tx, ty, yaw) of the costmap frame in the map frame."""
    grid = np.asarray(data, dtype=np.uint8).reshape(size_y, size_x)
    jj, ii = np.nonzero(grid == lethal)
    ij = np.stack([ii, jj], axis=1).astype(int)
    local = (ij + 0.5) * resolution
    cos_o, sin_o = math.cos(origin_yaw), math.sin(origin_yaw)
    in_frame = np.stack([origin_xy[0] + cos_o * local[:, 0] - sin_o * local[:, 1],
                         origin_xy[1] + sin_o * local[:, 0] + cos_o * local[:, 1]], axis=1)
    tx, ty, yaw = tf_map_costmap
    cos_t, sin_t = math.cos(yaw), math.sin(yaw)
    xy = np.stack([tx + cos_t * in_frame[:, 0] - sin_t * in_frame[:, 1],
                   ty + sin_t * in_frame[:, 0] + cos_t * in_frame[:, 1]], axis=1)
    return ij, xy


def static_mask(xy, occupied, resolution, origin_xy, margin):
    """True for every point whose distance to an occupied map cell center is
    <= margin. occupied: bool (H, W) array indexed [row=y, col=x]. Points off the
    map are not static."""
    h, w = occupied.shape
    r = int(math.ceil(margin / resolution)) + 1
    out = np.zeros(len(xy), dtype=bool)
    for k, (x, y) in enumerate(xy):
        ci = int(math.floor((x - origin_xy[0]) / resolution))
        cj = int(math.floor((y - origin_xy[1]) / resolution))
        j0, j1 = max(cj - r, 0), min(cj + r + 1, h)
        i0, i1 = max(ci - r, 0), min(ci + r + 1, w)
        if j0 >= j1 or i0 >= i1:
            continue
        oj, oi = np.nonzero(occupied[j0:j1, i0:i1])
        if len(oj) == 0:
            continue
        cx = origin_xy[0] + (oi + i0 + 0.5) * resolution
        cy = origin_xy[1] + (oj + j0 + 0.5) * resolution
        out[k] = bool(np.any(np.hypot(cx - x, cy - y) <= margin + 1e-9))
    return out


def mark_mask(xy, marks, margin):
    """True for every point inside a mark square (half side + margin).
    marks: [(x, y, side)], axis-aligned squares centred on (x, y)."""
    out = np.zeros(len(xy), dtype=bool)
    for mx, my, side in marks:
        half = side / 2.0 + margin + 1e-9
        out |= (np.abs(xy[:, 0] - mx) <= half) & (np.abs(xy[:, 1] - my) <= half)
    return out


def _shift(g, di, dj):
    """g moved by (di, dj) cells (+x, +y), False fill."""
    out = np.zeros_like(g)
    h, w = g.shape
    out[max(dj, 0):h + min(dj, 0), max(di, 0):w + min(di, 0)] = \
        g[max(-dj, 0):h + min(-dj, 0), max(-di, 0):w + min(-di, 0)]
    return out


def gap_merged_cells(cells_xy, marks, resolution, merge_gap):
    """Static-free lethal cells joined to a mark through gaps <= merge_gap.
    Local map-frame grid on multiples of resolution (stable cell centers from tick
    to tick); marks [(x, y, side)] are filled squares. Cells within r = merge_gap / 2
    are dilated, 8-connected components of that are the groups "<= merge_gap apart";
    groups holding a mark cell are kept.
    Returns (extend Nx2: grid-snapped centers of kept input cells outside every
    mark square, in_kept bool per input cell: it belongs to a kept group)."""
    cells_xy = np.asarray(cells_xy, dtype=float).reshape(-1, 2)
    in_kept = np.zeros(len(cells_xy), dtype=bool)
    empty = np.zeros((0, 2))
    if len(cells_xy) == 0 or not marks:
        return empty, in_kept
    lo = cells_xy.min(axis=0) - merge_gap
    hi = cells_xy.max(axis=0) + merge_gap
    near = [(mx, my, sd) for mx, my, sd in marks
            if lo[0] <= mx + sd / 2 and mx - sd / 2 <= hi[0]
            and lo[1] <= my + sd / 2 and my - sd / 2 <= hi[1]]
    if not near:
        return empty, in_kept
    rad = int(math.ceil(merge_gap / 2.0 / resolution))
    gi = np.floor(cells_xy / resolution).astype(int)
    pad = int(math.ceil(merge_gap / resolution)) + rad + 2
    base = gi.min(axis=0) - pad                       # grid index of local cell (0, 0)
    w, h = gi.max(axis=0) - base + pad + 1
    cx = (base[0] + np.arange(w) + 0.5) * resolution
    cy = (base[1] + np.arange(h) + 0.5) * resolution
    mark_grid = np.zeros((h, w), dtype=bool)
    for mx, my, sd in near:
        half = sd / 2.0 + 1e-9
        mark_grid |= ((np.abs(cx - mx) <= half)[None, :] & (np.abs(cy - my) <= half)[:, None])
    ij = gi - base
    occ = mark_grid.copy()
    occ[ij[:, 1], ij[:, 0]] = True

    offs = [(di, dj) for di in range(-rad, rad + 1) for dj in range(-rad, rad + 1)
            if math.hypot(di, dj) * resolution <= merge_gap / 2.0 + 1e-9]
    dil = np.zeros_like(occ)
    for di, dj in offs:
        dil |= _shift(occ, di, dj)
    jj, ii = np.nonzero(dil)
    labels = np.full(dil.shape, -1, dtype=int)
    labels[jj, ii] = cluster_cells(np.stack([ii, jj], axis=1))
    kept_labels = np.unique(labels[mark_grid])
    kept = np.isin(labels, kept_labels[kept_labels >= 0]) & dil
    in_kept = kept[ij[:, 1], ij[:, 0]]
    sel = in_kept & ~mark_grid[ij[:, 1], ij[:, 0]]
    ext = np.unique(np.stack([cx[ij[sel, 0]], cy[ij[sel, 1]]], axis=1), axis=0)
    return ext, in_kept


def cluster_cells(ij):
    """8-connected component labels (0..n-1) for integer grid cells ij (Nx2)."""
    index = {(int(a), int(b)): k for k, (a, b) in enumerate(ij)}
    labels = [-1] * len(ij)
    n = 0
    for start in range(len(ij)):
        if labels[start] >= 0:
            continue
        labels[start] = n
        stack = [start]
        while stack:
            a, b = ij[stack.pop()]
            for da in (-1, 0, 1):
                for db in (-1, 0, 1):
                    k = index.get((int(a) + da, int(b) + db))
                    if k is not None and labels[k] < 0:
                        labels[k] = n
                        stack.append(k)
        n += 1
    return np.array(labels, dtype=int)


def trim_path(path, robot_xy, trigger_dist):
    """Polyline from the point of the path closest to the robot, cut after
    trigger_dist of path length. Returns Mx2 (M >= 2) or None."""
    path = np.asarray(path, dtype=float)
    if len(path) < 2:
        return None
    best = None
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        seg = b - a
        ll = float(seg @ seg)
        t = 0.0 if ll == 0.0 else min(max(float((np.asarray(robot_xy) - a) @ seg) / ll, 0.0), 1.0)
        p = a + t * seg
        d = float(np.hypot(*(np.asarray(robot_xy) - p)))
        if best is None or d < best[0]:
            best = (d, i, p)
    _, i, p = best
    pts = [p] + [q for q in path[i + 1:]]
    out = [pts[0]]
    left = trigger_dist
    for q in pts[1:]:
        step = float(np.hypot(*(q - out[-1])))
        if step >= left:
            if step > 0.0:
                out.append(out[-1] + (q - out[-1]) * (left / step))
            break
        out.append(q)
        left -= step
    return np.array(out) if len(out) >= 2 else None


def project_on_path(path, pts):
    """Project points on a polyline. Returns (along, lat, outside): along-path
    distance of the closest point, its distance, and True for points whose closest
    point is the open end of the polyline (before the start / past the end)."""
    path = np.asarray(path, dtype=float)
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    seg = path[1:] - path[:-1]
    seglen = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seglen)])
    n_seg = len(seg)
    along = np.zeros(len(pts))
    lat = np.zeros(len(pts))
    outside = np.zeros(len(pts), dtype=bool)
    for k, p in enumerate(pts):
        best = None
        for i in range(n_seg):
            ll = seglen[i] ** 2
            t_raw = 0.0 if ll == 0.0 else float((p - path[i]) @ seg[i]) / ll
            t = min(max(t_raw, 0.0), 1.0)
            d = float(np.hypot(*(p - (path[i] + t * seg[i]))))
            if best is None or d < best[0]:
                best = (d, i, t, t_raw)
        d, i, t, t_raw = best
        along[k] = cum[i] + t * seglen[i]
        lat[k] = d
        outside[k] = (i == 0 and t_raw < -1e-9) or (i == n_seg - 1 and t_raw > 1.0 + 1e-9)
    return along, lat, outside


def track_sweep(tracked, face_mid, normal, now, assoc_radius, shift_left):
    """Record one finished sweep in tracked (list of dicts, mutated).
    face_mid / normal: swept face mid and unit normal (toward the robot) in the map
    frame. The sweep pushes along heading = -normal; pred = mid + left * shift_left,
    left = heading rotated +90 deg. A face mid within assoc_radius of an entry's mid
    or pred is the same object: count += 1 and mid/heading/pred follow the new sweep.
    Otherwise a new entry with count 1. Returns the entry."""
    mid = np.asarray(face_mid, dtype=float)
    n = np.asarray(normal, dtype=float)
    heading = -n / max(float(np.hypot(*n)), 1e-9)
    pred = mid + np.array([-heading[1], heading[0]]) * shift_left
    for e in tracked:
        if e.get("kind", "sweep") != "sweep":
            continue
        if min(float(np.hypot(*(mid - e["mid"]))),
               float(np.hypot(*(mid - e["pred"])))) <= assoc_radius:
            e["count"] += 1
            e.update(mid=mid, heading=heading, pred=pred, t=now)
            return e
    e = {"kind": "sweep", "mid": mid, "heading": heading, "pred": pred, "count": 1, "t": now}
    tracked.append(e)
    return e


def on_push_line(e, p, corridor_half, along_max):
    """True if point p lies on a push entry's approach line: perpendicular distance to
    the line through e["mid"] along e["heading"] <= corridor_half and the along-line
    coordinate from mid within [-0.3, along_max]."""
    h = e["heading"]
    d = np.asarray(p, dtype=float) - e["mid"]
    along = float(d[0] * h[0] + d[1] * h[1])
    perp = abs(float(d[0] * -h[1] + d[1] * h[0]))
    return perp <= corridor_half and -0.3 <= along <= along_max


def track_push(tracked, face_mid, normal, robot_xy, tip_offset, corridor_half, along_max,
               now):
    """Record one finished push in tracked (mutated). face_mid / normal: pushed face
    mid and unit normal (toward the robot); heading = -normal. The robot stands at the
    end of the push, so pred = robot + heading * tip_offset. A face mid on the approach
    line of an existing push entry is the same object: count += 1, heading/pred follow,
    the original mid stays. Otherwise a new entry with count 1. Returns the entry."""
    mid = np.asarray(face_mid, dtype=float)
    n = np.asarray(normal, dtype=float)
    heading = -n / max(float(np.hypot(*n)), 1e-9)
    pred = np.asarray(robot_xy, dtype=float) + heading * tip_offset
    for e in tracked:
        if e.get("kind") == "push" and on_push_line(e, mid, corridor_half, along_max):
            e["count"] += 1
            e.update(heading=heading, pred=pred, t=now)
            return e
    e = {"kind": "push", "mid": mid, "heading": heading, "pred": pred, "count": 1, "t": now}
    tracked.append(e)
    return e


def counts_of(e):
    """(sweep_count, push_count) to publish for a tracked entry (None = untracked)."""
    if e is None:
        return 0, 0
    return (e["count"], 0) if e.get("kind", "sweep") == "sweep" else (0, e["count"])


def classify_tracked(tracked, centroid_xy, corridor_half, assoc_radius, push_along=2.0):
    """(entry, outside) for a cluster centroid. Sweep entry = closest one whose pred or
    mid is within assoc_radius; outside = the centroid is more than corridor_half
    (perpendicular) from its approach line through mid along heading. Without a sweep
    match, a push entry whose approach line holds the centroid (on_push_line) is
    returned with outside False (pushed objects stay on the line, never skipped)."""
    c = np.asarray(centroid_xy, dtype=float)
    best = None
    for e in tracked:
        if e.get("kind", "sweep") != "sweep":
            continue
        d = min(float(np.hypot(*(c - e["mid"]))), float(np.hypot(*(c - e["pred"]))))
        if d <= assoc_radius and (best is None or d < best[0]):
            best = (d, e)
    if best is None:
        for e in tracked:
            if e.get("kind") == "push" and on_push_line(e, c, corridor_half, push_along):
                return e, False
        return None, False
    e = best[1]
    h = e["heading"]
    perp = abs(float((c[0] - e["mid"][0]) * -h[1] + (c[1] - e["mid"][1]) * h[0]))
    return e, perp > corridor_half


def add_ignored(ignored, centroid, width, now):
    """Record an ignored obstacle (list of dicts, mutated): radius = max(width/2, 0.15) + 0.15."""
    e = {"c": np.asarray(centroid, dtype=float), "radius": max(width / 2.0, 0.15) + 0.15,
         "t": now}
    ignored.append(e)
    return e


def prune_ignored(ignored, now, ttl):
    """Drop entries older than ttl seconds (in place)."""
    ignored[:] = [e for e in ignored if now - e["t"] <= ttl]


def is_ignored(ignored, centroid_xy):
    """True if the centroid lies within an ignored entry's radius."""
    c = np.asarray(centroid_xy, dtype=float)
    return any(float(np.hypot(*(c - e["c"]))) <= e["radius"] for e in ignored)


def pick_on_path(labels, xy, path, corridor_half, exclude=()):
    """Cluster on the path with the smallest along distance (labels in exclude are
    never picked). Returns (label, along, lateral) or None. path is the trimmed polyline."""
    if len(xy) == 0:
        return None
    along, lat, outside = project_on_path(path, xy)
    ok = ~outside & (lat <= corridor_half)
    if len(exclude):
        ok &= ~np.isin(labels, list(exclude))
    best = None
    for lab in np.unique(labels[ok]):
        sel = ok & (labels == lab)
        a = float(along[sel].min())
        if best is None or a < best[1]:
            best = (int(lab), a, float(lat[sel].min()))
    return best


def face_geometry(cells, robot_xy, resolution):
    """Visible front face of a cell cluster (map frame).
    Returns dict: centroid, start, end, polyline (<= 32 pts), width, normal
    (unit, toward the robot), degenerate."""
    cells = np.asarray(cells, dtype=float)
    centroid = cells.mean(axis=0)
    to_robot = np.asarray(robot_xy, dtype=float) - centroid
    norm = float(np.hypot(*to_robot))
    toward = to_robot / norm if norm > 1e-9 else np.array([1.0, 0.0])

    degenerate = len(cells) <= 2
    if not degenerate:
        evals, evecs = np.linalg.eigh(np.cov((cells - centroid).T, bias=True))
        degenerate = evals[1] / max(evals[0], 1e-12) < ROUND_EIGEN_RATIO
        axis = evecs[:, 1]
    if degenerate:
        normal = toward
        axis = np.array([-toward[1], toward[0]])
    else:
        normal = np.array([-axis[1], axis[0]])
        if float(normal @ toward) < 0.0:
            normal = -normal

    proj = (cells - centroid) @ axis
    order = np.argsort(proj)
    start = centroid + proj.min() * axis
    end = centroid + proj.max() * axis
    if degenerate:
        sub = cells[:: max(1, len(cells) // 256)]
        diff = sub[:, None, :] - sub[None, :, :]
        extent = float(np.sqrt((diff ** 2).sum(axis=2)).max())
    else:
        extent = float(proj.max() - proj.min())
    poly = cells[order]
    if len(poly) > MAX_FACE_POINTS:
        poly = poly[np.linspace(0, len(poly) - 1, MAX_FACE_POINTS).round().astype(int)]
    return {"centroid": centroid, "start": start, "end": end, "polyline": poly,
            "width": extent + resolution, "normal": normal, "degenerate": degenerate}


def find_path_obstacle(ij, xy, path, robot_xy, corridor_half, trigger_dist, resolution,
                       skip=None):
    """Static-free lethal cells (grid index ij, map xy) + full /plan polyline ->
    dict(face_geometry fields + along, lateral, n_cells) or None.
    skip(centroid_xy) -> True drops that cluster from the candidates."""
    trimmed = trim_path(path, robot_xy, trigger_dist)
    if trimmed is None or len(xy) == 0:
        return None
    labels = cluster_cells(ij)
    exclude = set()
    if skip is not None:
        exclude = {int(lab) for lab in np.unique(labels) if skip(xy[labels == lab].mean(axis=0))}
    hit = pick_on_path(labels, xy, trimmed, corridor_half, exclude)
    if hit is None:
        return None
    lab, along, lateral = hit
    cells = xy[labels == lab]
    geo = face_geometry(cells, robot_xy, resolution)
    geo.update(along=along, lateral=lateral, n_cells=int(len(cells)))
    return geo


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Point, Vector3
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid, Path
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from tf2_ros import Buffer, TransformListener, TransformException
    from mona_interfaces.msg import PathObstacle
    from obstacle_marks import goal_ended

    class PathObstacleMonitor(Node):
        def __init__(self):
            super().__init__("path_obstacle_monitor")
            self.static_margin = self.declare_parameter("static_margin", 0.15).value
            self.corridor_half = self.declare_parameter("corridor_half", 0.27).value
            self.trigger_dist = self.declare_parameter("trigger_dist", 2.5).value
            self.lethal = self.declare_parameter("lethal", 254).value
            self.merge_gap = self.declare_parameter("merge_gap", 0.44).value  # 2 x robot radius
            self.plan_timeout = self.declare_parameter("plan_timeout", 3.0).value
            self.map_frame = self.declare_parameter("map_frame", "map").value
            self.base_frame = self.declare_parameter("base_frame", "base_link").value
            self.assoc_radius = self.declare_parameter("assoc_radius", 0.3).value
            self.sweep_shift_left = self.declare_parameter("sweep_shift_left", 0.4).value
            self.push_tip_offset = self.declare_parameter("push_tip_offset", 0.20).value
            self.push_assoc_along = self.declare_parameter("push_assoc_along", 2.0).value

            self.costmap = None
            self.plan = None
            self.plan_time = 0.0
            self.marks = []      # [(x, y, side)] from /mona/obstacle_marks/list
            self.static = None   # (occupied bool HxW, resolution, origin_xy)
            self.tracked = []    # swept objects of this goal (track_sweep entries)
            self.ignored = []    # ignored obstacles (add_ignored entries)
            self.ignore_ttl = self.declare_parameter("ignore_ttl", 30.0).value
            self.seen_terminal = set()

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Costmap, "/local_costmap/costmap_raw",
                                     self._on_costmap, 1)
            self.create_subscription(Path, "/plan", self._on_plan, 1)
            self.create_subscription(
                OccupancyGrid, "/map", self._on_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(String, "/mona/obstacle_marks/list", self._on_marks, 1)
            self.create_subscription(PathObstacle, "/mona/swept_objects/add", self._on_swept, 10)
            self.create_subscription(PathObstacle, "/mona/pushed_objects/add", self._on_pushed, 10)
            self.create_subscription(PathObstacle, "/mona/ignored_objects/add",
                                     self._on_ignored, 10)
            self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                     self._on_status, 10)
            self.pub = self.create_publisher(PathObstacle, "/mona/path_obstacle", 1)
            self.pub_marker = self.create_publisher(Marker, "/mona/path_obstacle/marker", 1)
            self.pub_extend = self.create_publisher(String, "/mona/obstacle_marks/extend", 1)
            self.create_timer(0.2, self._tick)

        def _on_costmap(self, msg):
            self.costmap = msg

        def _on_plan(self, msg):
            self.plan = msg
            self.plan_time = time.monotonic()

        def _on_marks(self, msg):
            try:
                objs = json.loads(msg.data)["objects"]
                self.marks = [(float(o["x"]), float(o["y"]), float(o["width_m"])) for o in objs]
            except (ValueError, KeyError, TypeError):
                self.get_logger().warn("bad obstacle_marks list", throttle_duration_sec=5.0)

        def _on_swept(self, msg):
            mid = ((msg.face_start.x + msg.face_end.x) / 2.0,
                   (msg.face_start.y + msg.face_end.y) / 2.0)
            e = track_sweep(self.tracked, mid, (msg.normal.x, msg.normal.y),
                            time.monotonic(), self.assoc_radius, self.sweep_shift_left)
            self.get_logger().info(
                f"swept object at ({mid[0]:.2f}, {mid[1]:.2f}): sweep count {e['count']}")

        def _on_pushed(self, msg):
            mid = ((msg.face_start.x + msg.face_end.x) / 2.0,
                   (msg.face_start.y + msg.face_end.y) / 2.0)
            try:
                robot = self._lookup(self.map_frame, self.base_frame)
            except TransformException as ex:
                self.get_logger().warn(f"pushed object dropped, TF: {ex}")
                return
            e = track_push(self.tracked, mid, (msg.normal.x, msg.normal.y), robot[:2],
                           self.push_tip_offset, self.corridor_half, self.push_assoc_along,
                           time.monotonic())
            self.get_logger().info(
                f"pushed object at ({mid[0]:.2f}, {mid[1]:.2f}): push count {e['count']}, "
                f"pred ({e['pred'][0]:.2f}, {e['pred'][1]:.2f})")

        def _on_ignored(self, msg):
            add_ignored(self.ignored, (msg.centroid.x, msg.centroid.y), msg.width_m,
                        time.monotonic())
            self.get_logger().info(
                f"ignoring obstacle at ({msg.centroid.x:.2f}, {msg.centroid.y:.2f}) "
                f"for {self.ignore_ttl:.0f} s")

        def _on_status(self, msg):
            statuses = [(bytes(s.goal_info.goal_id.uuid).hex(), s.status)
                        for s in msg.status_list]
            if goal_ended(statuses, self.seen_terminal) and (self.tracked or self.ignored):
                self.tracked = []
                self.ignored = []
                self.get_logger().info("goal ended: swept-object tracking and ignore list cleared")

        def _skip(self, centroid):
            prune_ignored(self.ignored, time.monotonic(), self.ignore_ttl)
            if is_ignored(self.ignored, centroid):
                return True
            e, outside = classify_tracked(self.tracked, centroid, self.corridor_half,
                                          self.assoc_radius, self.push_assoc_along)
            if e is not None and outside:
                self.get_logger().info(
                    f"tracked object (sweeps {e['count']}) pushed out of its corridor: ignored",
                    throttle_duration_sec=5.0)
            return e is not None and outside

        def _on_map(self, msg):
            grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            self.static = (grid >= OCCUPIED_THRESH, msg.info.resolution,
                           (msg.info.origin.position.x, msg.info.origin.position.y))

        def _lookup(self, target, source):
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y,
                    _yaw_of(t.transform.rotation))

        def _detect(self):
            if (self.costmap is None or self.plan is None or self.static is None
                    or time.monotonic() - self.plan_time > self.plan_timeout
                    or len(self.plan.poses) < 2):
                return None
            cm = self.costmap
            try:
                tf_cm = self._lookup(self.map_frame, cm.header.frame_id)
                robot = self._lookup(self.map_frame, self.base_frame)
            except TransformException as e:
                self.get_logger().warn(f"TF: {e}", throttle_duration_sec=5.0)
                return None
            meta = cm.metadata
            ij, xy = costmap_cells_to_map(
                cm.data, meta.size_x, meta.size_y, meta.resolution,
                (meta.origin.position.x, meta.origin.position.y),
                _yaw_of(meta.origin.orientation), self.lethal, tf_cm)
            occupied, map_res, map_origin = self.static
            keep = ~static_mask(xy, occupied, map_res, map_origin, self.static_margin)
            if self.marks:
                keep &= ~mark_mask(xy, self.marks, self.static_margin)
                ext, in_kept = gap_merged_cells(xy[keep], self.marks, meta.resolution,
                                                self.merge_gap)
                keep[np.nonzero(keep)[0][in_kept]] = False
                if len(ext):
                    self.pub_extend.publish(String(data=json.dumps(
                        {"frame_id": self.map_frame, "res": meta.resolution,
                         "cells": np.round(ext, 3).tolist()}, separators=(",", ":"))))
            path = np.array([[p.pose.position.x, p.pose.position.y] for p in self.plan.poses])
            return find_path_obstacle(ij[keep], xy[keep], path, robot[:2],
                                      self.corridor_half, self.trigger_dist,
                                      meta.resolution, skip=self._skip), robot

        def _tick(self):
            result = self._detect()
            geo, robot = result if result is not None else (None, None)
            msg = PathObstacle()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.map_frame
            msg.source = "costmap_cells"
            msg.present = geo is not None
            if geo is not None:
                msg.centroid = Point(x=float(geo["centroid"][0]), y=float(geo["centroid"][1]))
                msg.face_start = Point(x=float(geo["start"][0]), y=float(geo["start"][1]))
                msg.face_end = Point(x=float(geo["end"][0]), y=float(geo["end"][1]))
                msg.face = [Point(x=float(p[0]), y=float(p[1])) for p in geo["polyline"]]
                msg.width_m = float(geo["width"])
                msg.normal = Vector3(x=float(geo["normal"][0]), y=float(geo["normal"][1]))
                msg.normal_degenerate = bool(geo["degenerate"])
                msg.along_m = float(geo["along"])
                msg.lateral_m = float(geo["lateral"])
                msg.n_cells = int(geo["n_cells"])
                e, _ = classify_tracked(self.tracked, geo["centroid"], self.corridor_half,
                                        self.assoc_radius, self.push_assoc_along)
                msg.sweep_count, msg.push_count = counts_of(e)
            self.pub.publish(msg)
            self._publish_marker(msg)

        def _publish_marker(self, msg):
            m = Marker()
            m.header = msg.header
            m.ns = "path_obstacle"
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.scale.x = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.0, 1.0
            m.pose.orientation.w = 1.0
            if not msg.present:
                m.action = Marker.DELETE
            else:
                m.action = Marker.ADD
                m.points = list(msg.face) if msg.face else [msg.centroid]
            self.pub_marker.publish(m)

    rclpy.init(args=args)
    node = PathObstacleMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
