#!/usr/bin/env python3
"""Temporary obstacle marks for the detour decision (static map is never updated).

Each /mona/obstacle_marks/add (mona_interfaces/PathObstacle, one per detour
decision, map frame) becomes one axis-aligned square mark: side = max(width_m,
0.1), the near edge on the observed face, extending behind it (the back depth is
unobservable, so the square assumes depth = face width). Marks expire after
ttl_s, and all of them are cleared when a NavigateToPose goal ends
(/navigate_to_pose/_action/status: a goal id not yet seen terminal reaching
SUCCEEDED / CANCELED / ABORTED).

/mona/obstacle_marks/extend (std_msgs/String JSON from path_obstacle_monitor:
{"frame_id", "res", "cells": [[x, y], ...]}) adds one res-sized mark per new grid
cell (fragments of the same obstacle joined to a mark); known
cells are not refreshed, so their TTL counts from the first add. The costmap
layer clamps side to >= 0.1 (+ padding); accepted.

/mona/obstacle_marks/remove (PathObstacle): removes the judged mark make_mark would
create for that msg (nearest centre within 0.2 m) and every extend cell added at or
after it (the BT pushed the object away, so the detour mark and its growth are void).

Publishes /mona/obstacle_marks/list (std_msgs/String, 1 Hz and on every change)
in the flat JSON that MovableObstacleLayer parses; path_obstacle_monitor reads
the same list to ignore marked obstacles.

Pure functions come first so they test without rclpy (test_obstacle_marks.py).
"""
import json
import math

MIN_SIDE = 0.1
TERMINAL = (4, 5, 6)   # action_msgs GoalStatus: SUCCEEDED, CANCELED, ABORTED


def make_mark(centroid, normal, width_m, now):
    """Square mark dict(x, y, side, t) from a PathObstacle's centroid, unit face
    normal (toward the robot) and width."""
    side = max(float(width_m), MIN_SIDE)
    return {"x": float(centroid[0]) - float(normal[0]) * side / 2.0,
            "y": float(centroid[1]) - float(normal[1]) * side / 2.0,
            "side": side, "t": float(now)}


def merge_extend(marks, cells, res, now):
    """Append a res-sized mark (with grid key) for every cell [x, y] whose
    key (floor(x / res), floor(y / res)) is not a mark yet. Returns the number added."""
    keys = {m["key"] for m in marks if "key" in m}
    added = 0
    for x, y in cells:
        key = (math.floor(x / res), math.floor(y / res))
        if key not in keys:
            keys.add(key)
            marks.append({"x": (key[0] + 0.5) * res, "y": (key[1] + 0.5) * res,
                          "side": float(res),
                          "t": float(now), "key": key})
            added += 1
    return added


def remove_mark(marks, centre, tol=0.2):
    """Remove (in place) the judged (non-extend) mark whose centre is nearest to `centre`
    within tol, plus every extend mark (has "key") added at or after that mark.
    Returns the number removed (0 if no mark matched)."""
    best = None
    for m in marks:
        if "key" in m:
            continue
        d = math.hypot(m["x"] - centre["x"], m["y"] - centre["y"])
        if d <= tol and (best is None or d < best[0]):
            best = (d, m)
    if best is None:
        return 0
    t0 = best[1]["t"]
    keep = [m for m in marks if m is not best[1] and not ("key" in m and m["t"] >= t0)]
    n = len(marks) - len(keep)
    marks[:] = keep
    return n


def split_expired(marks, now, ttl):
    """(alive, expired) marks; a mark lives for ttl seconds after its add time."""
    alive = [m for m in marks if now - m["t"] < ttl]
    return alive, [m for m in marks if now - m["t"] >= ttl]


def goal_ended(statuses, seen_terminal):
    """statuses: [(goal_id, status)] of one GoalStatusArray. Adds every terminal
    id to seen_terminal (set, mutated) and returns True if any of them was new."""
    ended = False
    for gid, status in statuses:
        if status in TERMINAL and gid not in seen_terminal:
            seen_terminal.add(gid)
            ended = True
    return ended


def marks_json(marks, frame_id):
    """Flat objects only: the C++ parser scans '{'..'}'."""
    objs = [{"x": m["x"], "y": m["y"], "width_m": m["side"],
             "movable": False, "pushable": False} for m in marks]
    return json.dumps({"frame_id": frame_id, "objects": objs}, separators=(",", ":"))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from action_msgs.msg import GoalStatusArray
    from std_msgs.msg import String
    from mona_interfaces.msg import PathObstacle

    class ObstacleMarks(Node):
        def __init__(self):
            super().__init__("obstacle_marks")
            self.ttl_s = self.declare_parameter("ttl_s", 120.0).value
            self.map_frame = self.declare_parameter("map_frame", "map").value
            self.marks = []
            self.seen_terminal = set()
            self.create_subscription(PathObstacle, "/mona/obstacle_marks/add", self._on_add, 10)
            self.create_subscription(PathObstacle, "/mona/obstacle_marks/remove",
                                     self._on_remove, 10)
            self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                     self._on_status, 10)
            self.create_subscription(String, "/mona/obstacle_marks/extend", self._on_extend, 10)
            self.pub = self.create_publisher(String, "/mona/obstacle_marks/list", 1)
            self.create_timer(1.0, self._tick)

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _publish(self):
            self.pub.publish(String(data=marks_json(self.marks, self.map_frame)))

        def _on_add(self, msg):
            if not msg.present:
                return
            m = make_mark((msg.centroid.x, msg.centroid.y), (msg.normal.x, msg.normal.y),
                          msg.width_m, self._now())
            self.marks.append(m)
            self.get_logger().info(
                f"add mark ({m['x']:.2f}, {m['y']:.2f}) side {m['side']:.2f}; "
                f"{len(self.marks)} marks")
            self._publish()

        def _on_remove(self, msg):
            m = make_mark((msg.centroid.x, msg.centroid.y), (msg.normal.x, msg.normal.y),
                          msg.width_m, 0.0)
            n = remove_mark(self.marks, m)
            self.get_logger().info(
                f"remove mark ({m['x']:.2f}, {m['y']:.2f}): {n} marks removed "
                f"(mark + later extends); {len(self.marks)} left")
            if n:
                self._publish()

        def _on_extend(self, msg):
            try:
                d = json.loads(msg.data)
                n = merge_extend(self.marks, d["cells"], float(d["res"]), self._now())
            except (ValueError, KeyError, TypeError):
                self.get_logger().warn("bad obstacle_marks extend", throttle_duration_sec=5.0)
                return
            if n:
                self.get_logger().info(f"extend {n} cells; {len(self.marks)} marks")
                self._publish()

        def _on_status(self, msg):
            statuses = [(bytes(s.goal_info.goal_id.uuid).hex(), s.status)
                        for s in msg.status_list]
            if goal_ended(statuses, self.seen_terminal) and self.marks:
                self.get_logger().info(f"goal ended: clear {len(self.marks)} marks")
                self.marks = []
                self._publish()

        def _tick(self):
            self.marks, expired = split_expired(self.marks, self._now(), self.ttl_s)
            if expired:
                self.get_logger().info(
                    f"expire {len(expired)} marks after {self.ttl_s:.0f} s; "
                    f"{len(self.marks)} left")
            self._publish()

    rclpy.init(args=args)
    node = ObstacleMarks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
