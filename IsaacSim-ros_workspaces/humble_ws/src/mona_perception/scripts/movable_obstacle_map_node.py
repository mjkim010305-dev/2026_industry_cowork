#!/usr/bin/env python3
"""Put the judged obstacles on the map.

vlm_movability_node measures every obstacle in the camera frame and lets the
model say what it is and whether it is movable. This node is the other half:
it transforms those points into the map frame, merges repeated sightings of
the same object - or, for the obstacle the VLM node has latched (track_id),
moves its entry to the latest measured position so a pushed object leaves no
trail - and publishes them three ways -

  /movable_obstacles/markers   MarkerArray  (cube + text, RViz)
  /movable_obstacles/poses     PoseArray    (movable ones only; frame = map)
  /movable_obstacles/list      String JSON  (everything, for the costmap layer
                                             and anything else programmatic)

- and keeps a semantic map on disk (semantic_map_path, YAML) that survives
restarts. Geometry comes from depth, names and verdicts from the model; this
node adds nothing of its own except the map frame and the bookkeeping.
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
import yaml


def _size_compatible(a, b, size_ratio):
    """Same size-gate rule as MovableObstacleMapNode used to apply via
    ``_same_size``, but as a free function so _remerge/_ingest need no Node
    instance (keeps them testable without rclpy)."""
    for key in ("width_m", "height_m"):
        big, small = max(a[key], b[key]), min(a[key], b[key])
        if big > 0.0 and (big - small) / big > size_ratio:
            return False
    return True


def _merge_pair(a, b, push_max_height, push_max_width, pinned_id=None):
    na, nb = a["n"], b["n"]
    total = na + nb
    # A pinned entry (the one _ingest is actively tracking) is the latest
    # real measurement of a moving object; averaging its position with a
    # stale sighting would pull it back toward the trail it just left - the
    # same bug the track-id move rule in _ingest exists to avoid. Its own
    # position/size/verdict wins outright, only n accumulates.
    pinned = a if a["id"] == pinned_id else (b if b["id"] == pinned_id else None)
    if pinned is not None:
        merged = dict(pinned)
        merged["n"] = total
        merged["first_seen"] = min(a.get("first_seen", 0.0), b.get("first_seen", 0.0))
        merged["last_seen"] = max(a.get("last_seen", 0.0), b.get("last_seen", 0.0))
        merged["pushable"] = bool(merged["movable"]
                                  and merged["height_m"] <= push_max_height
                                  and merged["width_m"] <= push_max_width)
        return merged
    merged = dict(a)
    for key in ("x", "y", "width_m", "height_m"):
        merged[key] = float((a[key] * na + b[key] * nb) / total)
    merged["n"] = total
    merged["id"] = min(a["id"], b["id"])
    merged["first_seen"] = min(a.get("first_seen", 0.0), b.get("first_seen", 0.0))
    merged["last_seen"] = max(a.get("last_seen", 0.0), b.get("last_seen", 0.0))
    # Label/movable follow the more recent sighting (tie -> more observations
    # behind it), the same latest-wins rule _ingest uses. Guarding against a
    # bad verdict is the VLM node's job - it only marks a candidate judged
    # after a k-vote quorum agrees - so this node does not need its own
    # conservative override on top of that.
    a_last, b_last = a.get("last_seen", 0.0), b.get("last_seen", 0.0)
    if a_last != b_last:
        newer = a if a_last > b_last else b
    else:
        newer = a if na >= nb else b
    merged["label"] = newer["label"]
    merged["movable"] = bool(newer["movable"])
    merged["pushable"] = bool(merged["movable"]
                              and merged["height_m"] <= push_max_height
                              and merged["width_m"] <= push_max_width)
    return merged


def _remerge(objects, merge_radius, merge_radius_tight, size_ratio,
             push_max_height, push_max_width, pinned_id=None):
    """Periodic re-merge for the drift `_ingest` cannot fix on its own:
    two sightings of the same object can land far enough apart on first
    arrival to become separate entries, then converge as their running
    averages settle - but `_ingest` only tests for a merge when a new
    sighting arrives, so a pair that has since drifted close never gets a
    chance to combine. Called from the save timer. Same two-tier
    distance/size rule as `_ingest`, same constants. Repeats passes until
    nothing merges, so chains (A~B, then B~C) collapse into one object.
    `pinned_id`, when set, is the entry the map node is actively tracking by
    track_id - see _merge_pair."""
    objects = list(objects)
    changed = True
    while changed:
        changed = False
        for i in range(len(objects)):
            for j in range(i + 1, len(objects)):
                a, b = objects[i], objects[j]
                d = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                if d < merge_radius and (d < merge_radius_tight
                                         or _size_compatible(a, b, size_ratio)):
                    objects[i] = _merge_pair(a, b, push_max_height, push_max_width,
                                             pinned_id)
                    del objects[j]
                    changed = True
                    break
            if changed:
                break
    return objects


def _usable_candidates(data, max_distance, keep_unjudged):
    """Candidates worth feeding to _ingest. `stale_ticks > 0` means the VLM
    node's track is lost and this message is a replay of its last good
    snapshot (every candidate in it, not just the latched one) - transformed
    through the robot's CURRENT pose it lands on old ground, which is how
    ghost map entries got created while a pushed box was out of view (t3
    postmortem). The whole message is discarded, not just the latched
    candidate, since the others are equally stale. Otherwise, the existing
    per-candidate distance/judged filter."""
    if data.get("stale_ticks", 0) > 0:
        return []
    out = []
    for c in data.get("candidates", []):
        if c.get("distance", 99.0) > max_distance:
            continue
        if not c.get("judged") and not keep_unjudged:
            continue
        out.append(c)
    return out


def _find_or_create(objects, next_id, mx, my, c, now, cfg):
    """Find a close, size-compatible entry for one observation, or start a
    new one, then advance it with a running average of position and size
    (latest verdict wins). Mutates `objects` in place (append on a new
    entry). This is the ordinary, no-track-id path; _ingest's track-id path
    replaces this with a straight move once a track is established."""
    best, best_d = None, cfg["merge_radius"]
    for o in objects:
        d = math.hypot(o["x"] - mx, o["y"] - my)
        if d < best_d and (d < cfg["merge_radius_tight"]
                            or _size_compatible(o, c, cfg["size_ratio"])):
            best, best_d = o, d
    if best is None:
        best = {"id": next_id, "x": mx, "y": my, "n": 0,
                "width_m": c["width_m"], "height_m": c["height_m"],
                "label": c["label"], "movable": c["movable"],
                "pushable": c["pushable"], "first_seen": now}
        next_id += 1
        objects.append(best)
    n = best["n"]
    for key, val in (("x", mx), ("y", my),
                     ("width_m", c["width_m"]), ("height_m", c["height_m"])):
        best[key] = (best[key] * n + val) / (n + 1)
    for key in ("x", "y", "width_m", "height_m"):
        best[key] = float(best[key])
    best["n"] = n + 1
    best["last_seen"] = float(now)
    if c.get("judged"):
        best["label"] = c["label"]
        best["movable"] = bool(c["movable"])
    best["pushable"] = bool(best["movable"]
                            and best["height_m"] <= cfg["push_max_height"]
                            and best["width_m"] <= cfg["push_max_width"])
    return best, next_id


def _ingest(objects, next_id, track, mx, my, c, now, cfg):
    """Advance the map with one observation. Mutates `objects` in place.
    `track` is `(track_id, entry_id)` for the object currently being
    followed, or None. `cfg` holds merge_radius, merge_radius_tight,
    size_ratio, push_max_height, push_max_width.

    c["track_id"] set (a latched VLM sighting) and it matches the entry we
    are already tracking: MOVE that entry to the latest measured position
    instead of averaging it in - a running average lags a pushed object
    behind, leaving its old position on the map, which is exactly how the
    t3 trail formed. Size still runs as a running average (short-term
    measurement noise, not drift).

    A track_id we have not associated yet, or whose entry has since
    disappeared (forgotten/merged away), falls back to the ordinary
    merge-or-create rule and starts a fresh association.

    track_id absent (an unlatched candidate): ordinary merge-or-create rule,
    unchanged, and `track` passes through untouched."""
    track_id = c.get("track_id")
    if track_id is not None and track is not None and track[0] == track_id:
        entry_id = track[1]
        entry = next((o for o in objects if o["id"] == entry_id), None)
        if entry is not None:
            n = entry["n"]
            entry["x"] = float(mx)
            entry["y"] = float(my)
            entry["width_m"] = float((entry["width_m"] * n + c["width_m"]) / (n + 1))
            entry["height_m"] = float((entry["height_m"] * n + c["height_m"]) / (n + 1))
            entry["n"] = n + 1
            entry["last_seen"] = float(now)
            if c.get("judged"):
                entry["label"] = c["label"]
                entry["movable"] = bool(c["movable"])
            entry["pushable"] = bool(entry["movable"]
                                     and entry["height_m"] <= cfg["push_max_height"]
                                     and entry["width_m"] <= cfg["push_max_width"])
            return next_id, track
        # The tracked entry is gone (forgotten or merged away) - re-associate.
    entry, next_id = _find_or_create(objects, next_id, mx, my, c, now, cfg)
    if track_id is not None:
        track = (track_id, entry["id"])
    return next_id, track


class MovableObstacleMapNode(Node):

    def __init__(self):
        super().__init__("movable_obstacle_map")
        self.target_frame = self.declare_parameter("target_frame", "map").value
        # Two sightings closer than this, of similar size, are one object.
        # 0.4 m: the carton and the forklift are 0.3 m apart at their nearest,
        # but differ 3x in height, so the size test keeps them separate.
        self.merge_radius = self.declare_parameter("merge_radius", 0.4).value
        self.merge_size_ratio = self.declare_parameter("merge_size_ratio", 0.5).value
        self.merge_radius_tight = self.declare_parameter("merge_radius_tight", 0.2).value
        # Verdict-less clusters (the model never saw them) are tracked but
        # not written to the semantic map unless this is on.
        self.keep_unjudged = self.declare_parameter("keep_unjudged", False).value
        self.semantic_map_path = self.declare_parameter(
            "semantic_map_path", os.path.expanduser("~/mona_semantic_map.yaml")).value
        self.save_period = self.declare_parameter("save_period", 5.0).value
        # Forget an object not seen for this long while its spot is in view.
        # 0 = never forget (a semantic map, not a tracker).
        self.forget_after = self.declare_parameter("forget_after", 0.0).value
        self.max_distance = self.declare_parameter("max_distance", 4.0).value
        # Pushability is re-derived from the AVERAGED size, with the same limits
        # vlm_movability_node uses per tick. A per-tick flag inherited from
        # the last sighting let a 0.92 m-tall cluster (the forklift's front,
        # mislabelled "Carton") stay pushable because one frame measured it
        # short.
        self.push_max_height = self.declare_parameter("push_max_height", 0.7).value
        self.push_max_width = self.declare_parameter("push_max_width", 1.2).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.objects = []          # list of dicts, the semantic map
        self.next_id = 1
        self.track = None          # (track_id, entry_id) currently followed
        self._load()

        self.markers_pub = self.create_publisher(MarkerArray, "/movable_obstacles/markers", 10)
        self.poses_pub = self.create_publisher(PoseArray, "/movable_obstacles/poses", 10)
        self.list_pub = self.create_publisher(String, "/movable_obstacles/list", 10)
        self.create_subscription(String, "/vlm/candidates", self._on_candidates, 10)
        self.create_timer(self.save_period, self._save)
        self.dirty = False
        self.get_logger().info(
            "movable_obstacle_map: %d objects loaded from %s, publishing in '%s'"
            % (len(self.objects), self.semantic_map_path, self.target_frame))

    # ------------------------------------------------------------------
    def _on_candidates(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        frame = data.get("frame_id", "")
        if not frame:
            return
        now = time.time()
        candidates = _usable_candidates(data, self.max_distance, self.keep_unjudged)
        if candidates:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame, rclpy.time.Time())
            except TransformException as exc:
                self.get_logger().warn("no TF %s <- %s: %s" % (self.target_frame, frame, exc),
                                       throttle_duration_sec=5.0)
                return
            for c in candidates:
                x, y, z = c["cam_xyz"]
                # Footprint centre: the measured point is on the front face; the
                # object extends about half its width behind it along the ray.
                half = 0.5 * float(c.get("width_m", 0.0))
                norm = math.sqrt(x * x + y * y + z * z) or 1.0
                pt = PointStamped()
                pt.header.frame_id = frame
                pt.point.x = x + x / norm * half
                pt.point.y = y + y / norm * half
                pt.point.z = z + z / norm * half
                pm = tf2_geometry_msgs.do_transform_point(pt, tf)
                # do_transform_point hands back numpy floats; YAML cannot represent them.
                self._observe(float(pm.point.x), float(pm.point.y), c, now)
        self._publish(now)

    def _observe(self, mx, my, c, now):
        """Merge with a known object or add a new one, or - for a candidate
        we are already tracking by track_id - move that object's entry to
        the latest position. See _ingest."""
        cfg = {"merge_radius": self.merge_radius, "merge_radius_tight": self.merge_radius_tight,
               "size_ratio": self.merge_size_ratio, "push_max_height": self.push_max_height,
               "push_max_width": self.push_max_width}
        prev = self.track
        before = {o["id"]: (o["x"], o["y"]) for o in self.objects}
        self.next_id, self.track = _ingest(self.objects, self.next_id, self.track,
                                           mx, my, c, now, cfg)
        self.dirty = True
        # Association and big moves of the followed entry leave a record, so a
        # run can show whether recovery re-attached to the same box and how far
        # the entry jumped.
        if c.get("track_id") is not None and self.track is not None:
            eid = self.track[1]
            if self.track != prev:
                self.get_logger().info("track %s -> entry %d at (%.2f, %.2f)"
                                       % (self.track[0], eid, mx, my))
            elif eid in before:
                jump = math.hypot(mx - before[eid][0], my - before[eid][1])
                if jump > self.merge_radius:
                    self.get_logger().info(
                        "track %s moved entry %d (%.2f, %.2f) -> (%.2f, %.2f), %.2f m"
                        % (self.track[0], eid, before[eid][0], before[eid][1], mx, my, jump))

    # ------------------------------------------------------------------
    def _publish(self, now):
        if self.forget_after > 0.0:
            self.objects = [o for o in self.objects
                            if now - o.get("last_seen", now) < self.forget_after]
        markers = MarkerArray()
        poses = PoseArray()
        poses.header.frame_id = self.target_frame
        poses.header.stamp = self.get_clock().now().to_msg()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for o in self.objects:
            cube = Marker()
            cube.header = poses.header
            cube.ns = "movable_obstacles"
            cube.id = o["id"]
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = o["x"]
            cube.pose.position.y = o["y"]
            cube.pose.position.z = o["height_m"] / 2.0
            cube.pose.orientation.w = 1.0
            side = max(0.1, o["width_m"])
            cube.scale.x = side
            cube.scale.y = side
            cube.scale.z = max(0.05, o["height_m"])
            # Same three colours as the camera overlay.
            if not o["movable"]:
                cube.color.r, cube.color.g, cube.color.b = 0.9, 0.25, 0.25
            elif o["pushable"]:
                cube.color.r, cube.color.g, cube.color.b = 0.3, 0.85, 0.3
            else:
                cube.color.r, cube.color.g, cube.color.b = 1.0, 0.75, 0.15
            cube.color.a = 0.6
            markers.markers.append(cube)
            text = Marker()
            text.header = poses.header
            text.ns = "movable_obstacles_text"
            text.id = o["id"]
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = o["x"]
            text.pose.position.y = o["y"]
            text.pose.position.z = o["height_m"] + 0.25
            text.scale.z = 0.18
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = "%s %s (%.1f,%.1f)" % (
                o["label"], "MOVABLE" if o["pushable"] else ("FIXED" if not o["movable"] else "TOO LARGE"),
                o["x"], o["y"])
            markers.markers.append(text)
            if o["pushable"]:
                pose = Pose()
                pose.position.x = o["x"]
                pose.position.y = o["y"]
                pose.orientation.w = 1.0
                poses.poses.append(pose)
        self.markers_pub.publish(markers)
        self.poses_pub.publish(poses)
        self.list_pub.publish(String(data=json.dumps({
            "frame_id": self.target_frame, "stamp": now,
            "objects": [{k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in o.items()} for o in self.objects],
        }, separators=(",", ":"))))

    # ------------------------------------------------------------------
    def _load(self):
        try:
            with open(self.semantic_map_path) as f:
                data = yaml.safe_load(f) or {}
        except OSError:
            return
        self.objects = data.get("objects", [])
        self.next_id = 1 + max([o["id"] for o in self.objects], default=0)

    def _save(self):
        # Runs on the save timer: catches objects that drifted together
        # after being split at first sighting (see _remerge docstring).
        pinned_id = self.track[1] if self.track else None
        merged = _remerge(self.objects, self.merge_radius, self.merge_radius_tight,
                          self.merge_size_ratio, self.push_max_height, self.push_max_width,
                          pinned_id)
        remerged = len(merged) != len(self.objects)
        if remerged:
            self.objects = merged
            self.dirty = True
        if not self.dirty:
            return
        tmp = self.semantic_map_path + ".tmp"
        with open(tmp, "w") as f:
            yaml.safe_dump({"frame_id": self.target_frame,
                            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "objects": self.objects}, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, self.semantic_map_path)
        self.dirty = False
        # Publish only after the file is safely on disk. At shutdown
        # (rclpy.spin's finally calls _save once more) the publishers may
        # already be torn down - a failure here must not look like the save
        # itself failed, since by this point it already succeeded.
        if remerged and rclpy.ok():
            try:
                self._publish(time.time())
            except Exception as exc:              # noqa: BLE001 - save already done
                self.get_logger().warn("post-remerge publish failed: %s" % exc)


def main(args=None):
    rclpy.init(args=args)
    node = MovableObstacleMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
