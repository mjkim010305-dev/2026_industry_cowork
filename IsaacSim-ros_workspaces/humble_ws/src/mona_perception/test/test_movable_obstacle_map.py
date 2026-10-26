#!/usr/bin/env python3
"""Unit tests for movable_obstacle_map_node's pure functions: re-merge
(_remerge), the stale-message filter (_usable_candidates) and track-follow
ingest (_ingest).

Import only - no rclpy node is constructed, no ROS graph is touched.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from movable_obstacle_map_node import (  # noqa: E402
    _remerge, _usable_candidates, _ingest,
)

MERGE_RADIUS = 0.4
MERGE_RADIUS_TIGHT = 0.2
SIZE_RATIO = 0.5
PUSH_MAX_HEIGHT = 0.7
PUSH_MAX_WIDTH = 1.2
CFG = {"merge_radius": MERGE_RADIUS, "merge_radius_tight": MERGE_RADIUS_TIGHT,
       "size_ratio": SIZE_RATIO, "push_max_height": PUSH_MAX_HEIGHT,
       "push_max_width": PUSH_MAX_WIDTH}


def _obj(id, x, y, width_m, height_m, n=1, movable=True, label="box",
         first_seen=0.0, last_seen=0.0, pushable=True):
    return {"id": id, "x": x, "y": y, "width_m": width_m, "height_m": height_m,
            "n": n, "movable": movable, "label": label, "pushable": pushable,
            "first_seen": first_seen, "last_seen": last_seen}


def _run(objects):
    return _remerge(list(objects), MERGE_RADIUS, MERGE_RADIUS_TIGHT, SIZE_RATIO,
                     PUSH_MAX_HEIGHT, PUSH_MAX_WIDTH)


def test_similar_size_within_merge_radius_merges():
    # 0.3 m apart (< 0.4 merge_radius, >= 0.2 tight), similar size -> the
    # size gate must pass too, and it does (widths/heights close).
    a = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4, n=2, first_seen=1.0, last_seen=5.0)
    b = _obj(2, x=0.3, y=0.0, width_m=0.55, height_m=0.42, n=1, first_seen=2.0, last_seen=6.0)
    result = _run([a, b])
    assert len(result) == 1
    merged = result[0]
    # Hand-computed n-weighted average (n=2 and n=1, total 3).
    assert merged["x"] == (0.0 * 2 + 0.3 * 1) / 3
    assert merged["width_m"] == (0.5 * 2 + 0.55 * 1) / 3
    assert merged["height_m"] == (0.4 * 2 + 0.42 * 1) / 3
    assert merged["n"] == 3
    assert merged["id"] == 1
    assert merged["first_seen"] == 1.0
    assert merged["last_seen"] == 6.0


def test_different_size_within_tight_radius_still_merges():
    # 0.15 m apart (< 0.2 tight radius): the tight tier ignores size
    # entirely, the way _observe does for the same reason (a single
    # object's measured width can swing as the robot approaches).
    a = _obj(1, x=0.0, y=0.0, width_m=0.4, height_m=0.3, n=1)
    b = _obj(2, x=0.15, y=0.0, width_m=1.0, height_m=1.5, n=1)
    result = _run([a, b])
    assert len(result) == 1


def test_dissimilar_size_outside_tight_radius_stays_separate():
    # 0.3 m apart: inside merge_radius but outside the tight radius, so
    # the size gate applies. Box 0.5x0.4 vs forklift 1.2x2.0 fails it.
    box = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4)
    forklift = _obj(2, x=0.3, y=0.0, width_m=1.2, height_m=2.0)
    result = _run([box, forklift])
    assert len(result) == 2


def test_outside_merge_radius_stays_separate():
    a = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4)
    b = _obj(2, x=0.5, y=0.0, width_m=0.5, height_m=0.4)
    result = _run([a, b])
    assert len(result) == 2


def test_movable_conflict_adopts_more_recent_verdict():
    # Close together, similar size, but the two sightings disagree on
    # movable. The earlier sighting (n=5) says movable; the later one
    # (n=1, higher last_seen) says fixed. Latest-wins: the merged verdict
    # follows the later sighting regardless of n, the same rule _observe
    # applies to a single object's running verdict.
    a = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4, n=5, movable=True,
             last_seen=1.0)
    b = _obj(2, x=0.1, y=0.0, width_m=0.52, height_m=0.41, n=1, movable=False,
             last_seen=9.0)
    result = _run([a, b])
    assert len(result) == 1
    assert result[0]["movable"] is False
    assert result[0]["pushable"] is False


def test_movable_conflict_tie_breaks_on_n():
    # Same last_seen: the sighting with more observations behind it wins.
    a = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4, n=5, movable=True,
             last_seen=5.0)
    b = _obj(2, x=0.1, y=0.0, width_m=0.52, height_m=0.41, n=1, movable=False,
             last_seen=5.0)
    result = _run([a, b])
    assert len(result) == 1
    assert result[0]["movable"] is True


def test_pushable_recomputed_from_merged_average_size():
    # Both sightings are individually flagged pushable, but their
    # n-weighted average width crosses push_max_width (1.2): (1.1*1 +
    # 1.35*1) / 2 = 1.225 > 1.2. Merged pushable must come from the
    # recomputed average size, not be inherited from either sighting's
    # own flag.
    a = _obj(1, x=0.0, y=0.0, width_m=1.1, height_m=0.5, n=1, movable=True,
             pushable=True, last_seen=1.0)
    b = _obj(2, x=0.1, y=0.0, width_m=1.35, height_m=0.5, n=1, movable=True,
             pushable=True, last_seen=2.0)
    result = _run([a, b])
    assert len(result) == 1
    merged = result[0]
    assert merged["width_m"] == (1.1 * 1 + 1.35 * 1) / 2
    assert merged["movable"] is True
    assert merged["pushable"] is False


def test_chain_merge_collapses_all_three():
    # A-B close, B-C close, A-C far: still one object after the full pass.
    a = _obj(1, x=0.0, y=0.0, width_m=0.5, height_m=0.4)
    b = _obj(2, x=0.15, y=0.0, width_m=0.5, height_m=0.4)
    c = _obj(3, x=0.3, y=0.0, width_m=0.5, height_m=0.4)
    result = _run([a, b, c])
    assert len(result) == 1


def _cand(track_id=None, width_m=0.5, height_m=0.4, label="box", movable=True,
          judged=True, pushable=True, distance=1.0):
    return {"track_id": track_id, "width_m": width_m, "height_m": height_m,
            "label": label, "movable": movable, "judged": judged,
            "pushable": pushable, "distance": distance}


# -- _usable_candidates ------------------------------------------------------

def test_usable_candidates_stale_message_discarded_entirely():
    data = {"stale_ticks": 3, "candidates": [_cand(judged=True, distance=1.0)]}
    assert _usable_candidates(data, max_distance=4.0, keep_unjudged=False) == []


def test_usable_candidates_fresh_message_applies_distance_and_judged_filter():
    box = _cand(judged=True, distance=1.0)
    unjudged = _cand(judged=False, distance=1.0)
    far = _cand(judged=True, distance=5.0)
    data = {"stale_ticks": 0, "candidates": [box, unjudged, far]}
    result = _usable_candidates(data, max_distance=4.0, keep_unjudged=False)
    assert result == [box]


# -- _ingest ------------------------------------------------------------------

def test_ingest_same_track_id_moves_entry_to_latest_position():
    objects = []
    c1 = _cand(track_id="T", width_m=0.75, height_m=0.43)
    next_id, track = _ingest(objects, 1, None, 30.48, -3.62, c1, 0.0, CFG)
    assert len(objects) == 1
    assert track == ("T", 1)

    c2 = _cand(track_id="T", width_m=0.56, height_m=0.23)
    next_id, track = _ingest(objects, next_id, track, 31.35, -3.54, c2, 1.0, CFG)
    assert len(objects) == 1
    entry = objects[0]
    # Position: the latest measurement, not an average with the old spot.
    assert entry["x"] == 31.35
    assert entry["y"] == -3.54
    # Size: still a running average.
    assert entry["n"] == 2
    assert entry["width_m"] == 0.655
    assert entry["height_m"] == 0.33
    assert track == ("T", 1)


def test_ingest_new_track_id_starts_new_entry_leaves_old_one_alone():
    objects = []
    next_id, track = _ingest(objects, 1, None, 30.48, -3.62,
                             _cand(track_id="T", width_m=0.75, height_m=0.43), 0.0, CFG)
    next_id, track = _ingest(objects, next_id, track, 31.35, -3.54,
                             _cand(track_id="T", width_m=0.56, height_m=0.23), 1.0, CFG)
    next_id, track = _ingest(objects, next_id, track, 34.54, -4.50,
                             _cand(track_id="T2", width_m=0.65, height_m=0.25), 2.0, CFG)
    assert len(objects) == 2
    old = next(o for o in objects if o["id"] == 1)
    assert old["x"] == 31.35 and old["y"] == -3.54
    assert track[0] == "T2"
    new = next(o for o in objects if o["id"] == track[1])
    assert new["x"] == 34.54 and new["y"] == -4.50


def test_ingest_tracked_entry_gone_reassociates_without_error():
    objects = []
    next_id, track = _ingest(objects, 1, None, 30.48, -3.62,
                             _cand(track_id="T", width_m=0.75, height_m=0.43), 0.0, CFG)
    assert track == ("T", 1)
    objects.clear()  # entry forgotten/merged away
    next_id, track = _ingest(objects, next_id, track, 31.35, -3.54,
                             _cand(track_id="T", width_m=0.56, height_m=0.23), 1.0, CFG)
    assert len(objects) == 1
    assert track == ("T", objects[0]["id"])
    assert objects[0]["x"] == 31.35 and objects[0]["y"] == -3.54


def test_ingest_no_track_id_keeps_running_average_behaviour():
    objects = []
    next_id, track = _ingest(objects, 1, None, 0.0, 0.0, _cand(track_id=None), 0.0, CFG)
    next_id, track = _ingest(objects, next_id, track, 0.1, 0.0, _cand(track_id=None), 1.0, CFG)
    assert len(objects) == 1
    assert objects[0]["x"] == 0.05
    assert objects[0]["n"] == 2
    assert track is None


def test_ingest_track_move_recomputes_pushable_from_new_average_height():
    objects = []
    next_id, track = _ingest(objects, 1, None, 0.0, 0.0,
                             _cand(track_id="T", width_m=0.5, height_m=0.6, movable=True),
                             0.0, CFG)
    assert objects[0]["pushable"] is True
    next_id, track = _ingest(objects, next_id, track, 0.05, 0.0,
                             _cand(track_id="T", width_m=0.5, height_m=0.9, movable=True),
                             1.0, CFG)
    assert objects[0]["height_m"] == 0.75
    assert objects[0]["pushable"] is False


# -- _remerge pinned_id -------------------------------------------------------

def test_remerge_pinned_entry_wins_position_over_averaging():
    stale = _obj(2, x=32.40, y=-3.70, width_m=0.5, height_m=0.4, n=10)
    pinned = _obj(5, x=32.52, y=-3.74, width_m=0.5, height_m=0.4, n=5)
    result = _remerge([stale, pinned], MERGE_RADIUS, MERGE_RADIUS_TIGHT, SIZE_RATIO,
                      PUSH_MAX_HEIGHT, PUSH_MAX_WIDTH, pinned_id=5)
    assert len(result) == 1
    merged = result[0]
    assert merged["id"] == 5
    assert merged["x"] == 32.52
    assert merged["y"] == -3.74
    assert merged["n"] == 15


def test_remerge_without_pinned_id_falls_back_to_weighted_average():
    stale = _obj(2, x=32.40, y=-3.70, width_m=0.5, height_m=0.4, n=10)
    other = _obj(5, x=32.52, y=-3.74, width_m=0.5, height_m=0.4, n=5)
    result = _remerge([stale, other], MERGE_RADIUS, MERGE_RADIUS_TIGHT, SIZE_RATIO,
                      PUSH_MAX_HEIGHT, PUSH_MAX_WIDTH)
    assert len(result) == 1
    assert result[0]["x"] == (32.40 * 10 + 32.52 * 5) / 15
