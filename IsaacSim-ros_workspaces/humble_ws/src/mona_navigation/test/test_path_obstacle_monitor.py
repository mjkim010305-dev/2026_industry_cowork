#!/usr/bin/env python3
"""Unit tests for path_obstacle_monitor's pure functions on a synthetic costmap.

Grid: 100 x 100 cells, 0.05 m, origin (-1.0, -2.5), so cell (i, j) has its
center at x = -1 + (i + 0.5) * 0.05, y = -2.5 + (j + 0.5) * 0.05. Robot at the
origin, path along +x. Expected values are written out by hand.

Import only - no rclpy node is constructed, no ROS graph is touched.
"""
import math
import os
import sys

import numpy as np
from pytest import approx

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from path_obstacle_monitor import (  # noqa: E402
    costmap_cells_to_map, static_mask, find_path_obstacle, mark_mask, gap_merged_cells,
    track_sweep, track_push, counts_of, classify_tracked, add_ignored, prune_ignored, is_ignored,
)

RES = 0.05
ORIGIN = (-1.0, -2.5)
SIZE = 100
MARGIN = 0.15
CORRIDOR = 0.27
TRIGGER = 2.5
LETHAL = 254
ROBOT = (0.0, 0.0)
PATH_X = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]


def box(i0, i1, j0, j1):
    return [(i, j) for i in range(i0, i1 + 1) for j in range(j0, j1 + 1)]


def center(i, j):
    return (ORIGIN[0] + (i + 0.5) * RES, ORIGIN[1] + (j + 0.5) * RES)


def run(cells, static_cells=(), path=PATH_X, robot=ROBOT, tf=(0.0, 0.0, 0.0),
        trigger=TRIGGER):
    data = np.zeros((SIZE, SIZE), dtype=np.uint8)
    data[0, 0] = 253   # inflation value next to lethal must be ignored
    for i, j in cells:
        data[j, i] = LETHAL
    ij, xy = costmap_cells_to_map(data.ravel(), SIZE, SIZE, RES, ORIGIN, 0.0, LETHAL, tf)
    occupied = np.zeros((SIZE, SIZE), dtype=bool)
    for i, j in static_cells:
        occupied[j, i] = True
    # static map shares the costmap grid only in the identity-tf cases
    keep = ~static_mask(xy, occupied, RES, ORIGIN, MARGIN)
    return find_path_obstacle(ij[keep], xy[keep], np.array(path), robot, CORRIDOR, trigger, RES)


def test_1_box_on_path_present():
    # 3 x 5 cells: x = 2.025..2.125, y = -0.075..0.125 -> centroid (2.075, 0.025)
    geo = run(box(60, 62, 48, 52))
    assert geo is not None
    assert geo["centroid"] == approx((2.075, 0.025), abs=1e-6)
    assert geo["along"] == approx(2.025, abs=1e-6)      # nearest cell column x = 2.025
    assert geo["lateral"] == approx(0.025, abs=1e-6)    # cells at y = +-0.025 are closest
    assert geo["n_cells"] == 15


def test_2_static_wall_excluded():
    wall = box(60, 60, 40, 60)
    assert run(wall, static_cells=wall) is None


def test_3_static_offset():
    static = [(60, 50)]
    # 2 cells = 0.10 m from the static cell: AMCL offset, excluded
    assert run([(62, 50)], static_cells=static) is None
    # 6 cells = 0.30 m away: a real obstacle, x = -1 + 66.5 * 0.05 = 2.325
    geo = run([(66, 50)], static_cells=static)
    assert geo is not None
    assert geo["centroid"] == approx((2.325, 0.025), abs=1e-6)


def test_4_lateral_040_not_detected():
    # j = 58 -> y = 0.425 (> corridor 0.27)
    assert run(box(60, 62, 58, 58)) is None


def test_5_beyond_path_end_not_detected():
    short = [(0.0, 0.0), (2.0, 0.0)]
    assert run([(65, 50)], path=short) is None                    # x = 2.275 past the end
    assert run([(79, 50)], path=PATH_X) is None                   # x = 2.975 past trigger_dist
    assert run([(59, 50)], path=PATH_X) is not None               # x = 1.975 inside


def test_6_map_odom_offset_and_rotation():
    # odom -> map: rotate +90 deg, then shift (10, 5): (x, y) -> (10 - y, 5 + x)
    path = [(10.0, 5.0 + k) for k in range(5)]
    geo = run(box(60, 62, 49, 51), path=path, robot=(10.0, 5.0), tf=(10.0, 5.0, math.pi / 2))
    assert geo is not None
    # odom centroid (2.075, 0.025) -> map (9.975, 7.075); nearest cell x_odom 2.025 -> along 2.025
    assert geo["centroid"] == approx((9.975, 7.075), abs=1e-6)
    assert geo["along"] == approx(2.025, abs=1e-6)
    assert geo["lateral"] == approx(0.025, abs=1e-6)


def test_7_face_line_geometry():
    # 14 cells at x = 2.025, y = -0.325 .. 0.325 (perpendicular to the path)
    geo = run([(60, j) for j in range(43, 57)])
    assert geo is not None
    assert not geo["degenerate"]
    assert abs(geo["width"] - 0.70) <= 0.05
    assert geo["normal"][0] == approx(-1.0, abs=1e-6)
    assert geo["normal"][1] == approx(0.0, abs=1e-6)
    ys = sorted([geo["start"][1], geo["end"][1]])
    assert abs(ys[0] - (-0.325)) <= 0.05 and abs(ys[1] - 0.325) <= 0.05
    assert abs(geo["start"][0] - 2.025) <= 0.05 and abs(geo["end"][0] - 2.025) <= 0.05
    py = [p[1] for p in geo["polyline"]]
    assert py == sorted(py) or py == sorted(py, reverse=True)


def test_7b_polyline_capped_at_32():
    geo = run([(60, j) for j in range(0, 100)])
    assert geo is not None
    assert len(geo["polyline"]) == 32


def test_8_degenerate_single_cell_and_L():
    # single cell (2.025, 0.025); robot at origin
    geo = run([(60, 50)])
    n = math.hypot(2.025, 0.025)
    assert geo["degenerate"]
    assert geo["normal"] == approx((-2.025 / n, -0.025 / n), abs=1e-6)

    # thick L (5-cell-wide arms: eigenvalue ratio 1.92 < 2; a 3-wide L is 2.5 and
    # keeps its line fit): horizontal arm i 60..68 x j 48..52, vertical arm i 60..64 x j 53..56
    cells = box(60, 68, 48, 52) + box(60, 64, 53, 56)
    mean = np.mean([center(i, j) for i, j in cells], axis=0)
    toward = np.array(ROBOT) - mean
    toward = toward / np.hypot(*toward)
    geo = run(cells)
    assert geo is not None
    assert geo["degenerate"]
    assert geo["centroid"] == approx(tuple(mean), abs=1e-6)
    assert geo["normal"] == approx(tuple(toward), abs=1e-6)


def test_9_nearest_of_several_clusters():
    far = box(79, 81, 49, 51)      # x ~ 3.0
    near = box(49, 51, 49, 51)     # x ~ 1.5
    geo = run(far + near, trigger=4.0)
    assert geo is not None
    assert geo["centroid"][0] == approx(1.525, abs=1e-6)
    assert geo["along"] == approx(1.475, abs=1e-6)


def test_10_mark_exclusion():
    # mark square centre (2.075, 0.025), side 0.2 -> half 0.1 + margin 0.15 = 0.25
    marks = [(2.075, 0.025, 0.2)]
    xy = np.array([[2.075, 0.025], [2.30, 0.20], [2.40, 0.025], [2.075, 0.30]])
    assert list(mark_mask(xy, marks, MARGIN)) == [True, True, False, False]
    assert not mark_mask(xy, [], MARGIN).any()
    # the box of test 1 is fully inside -> no obstacle once marked
    cells = box(60, 62, 48, 52)
    ij, cxy = costmap_cells_to_map(
        np.where(np.isin(np.arange(SIZE * SIZE), [j * SIZE + i for i, j in cells]),
                 LETHAL, 0).astype(np.uint8), SIZE, SIZE, RES, ORIGIN, 0.0, LETHAL,
        (0.0, 0.0, 0.0))
    keep = ~mark_mask(cxy, marks, MARGIN)
    assert find_path_obstacle(ij[keep], cxy[keep], np.array(PATH_X), ROBOT, CORRIDOR,
                              TRIGGER, RES) is None


def _row(x0, x1, y, step=0.05):
    return np.array([[x, y] for x in np.arange(x0, x1 + 1e-9, step)])


def _snapped(xy):
    return {(math.floor(x / RES), math.floor(y / RES)) for x, y in xy}


def test_11_gap_merge_chain():
    marks = [(0.0, 0.0, 0.4)]                       # covers x, y in [-0.2, 0.2]
    # single-cell-thick fragments 0.1 m long with 0.3 m gaps: x 0.5-0.6, 0.9-1.0 (the first
    # is 0.3 m from the mark)
    frag = np.vstack([_row(0.5, 0.6, 0.0), _row(0.9, 1.0, 0.0)])
    far = _row(0.0, 0.1, 1.0)                        # 0.8 m from everything
    cells = np.vstack([frag, far])
    ext, kept = gap_merged_cells(cells, marks, RES, 0.44)
    assert kept[:len(frag)].all() and not kept[len(frag):].any()
    assert _snapped(ext) == _snapped(frag)           # exactly the fragments, no fill
    assert not np.any(mark_mask(ext, marks, 0.0))


def test_12_only_input_cells_and_no_marks():
    marks = [(0.0, 0.0, 0.4)]
    cells = _row(0.5, 1.5, 0.0)
    ext, kept = gap_merged_cells(cells, marks, RES, 0.44)
    assert kept.all()
    assert _snapped(ext) <= _snapped(cells)
    ext, kept = gap_merged_cells(cells, [], RES, 0.44)
    assert len(ext) == 0 and not kept.any()


def test_13_extension_cells_stable_across_ticks():
    marks = [(0.0, 0.0, 0.4)]
    cells = _row(0.5, 0.6, 0.0) + 0.025   # cell centers
    shifted = np.vstack([cells + [0.013, -0.011], [[1.0, 1.0]]])   # sub-cell shift, other bbox
    e1, _ = gap_merged_cells(cells, marks, RES, 0.44)
    e2, _ = gap_merged_cells(shifted, marks, RES, 0.44)
    assert {tuple(np.round(p, 6)) for p in e1} == {tuple(np.round(p, 6)) for p in e2}


# --- sweep tracking (F1) ---
ASSOC = 0.3
SHIFT = 0.4
S36E_MID = (30.60, -3.50)


def s36e_tracked():
    tracked = []
    track_sweep(tracked, S36E_MID, (-1.0, 0.0), 0.0, ASSOC, SHIFT)
    return tracked


def test_14_track_sweep_entry_fields():
    e = s36e_tracked()[0]
    assert e["count"] == 1
    assert tuple(e["heading"]) == approx((1.0, 0.0))
    assert tuple(e["pred"]) == approx((30.60, -3.10))   # left of +x is +y


def test_15_second_sweep_same_object_increments():
    tracked = s36e_tracked()
    track_sweep(tracked, (30.75, -3.12), (-1.0, 0.0), 1.0, ASSOC, SHIFT)  # near pred
    assert len(tracked) == 1 and tracked[0]["count"] == 2
    assert tuple(tracked[0]["mid"]) == approx((30.75, -3.12))
    track_sweep(tracked, (30.70, -3.40), (-1.0, 0.0), 2.0, ASSOC, SHIFT)  # near old mid
    assert len(tracked) == 1 and tracked[0]["count"] == 3


def test_16_other_object_gets_own_entry():
    tracked = s36e_tracked()
    track_sweep(tracked, (30.60, -2.50), (-1.0, 0.0), 1.0, ASSOC, SHIFT)  # 1 m away
    assert len(tracked) == 2 and [e["count"] for e in tracked] == [1, 1]


def test_17_classify_s36e_numbers():
    tracked = s36e_tracked()
    e, outside = classify_tracked(tracked, (30.77, -3.14), CORRIDOR, ASSOC)
    assert e is tracked[0] and outside          # perpendicular 0.36 > 0.27
    e, outside = classify_tracked(tracked, (30.62, -3.48), CORRIDOR, ASSOC)
    assert e is tracked[0] and not outside      # near mid, inside the corridor
    e, outside = classify_tracked(tracked, (31.5, -3.0), CORRIDOR, ASSOC)
    assert e is None and not outside


def test_18_excluded_cluster_not_picked_farther_one_is():
    ij = np.array(box(20, 21, 49, 50) + box(40, 41, 49, 50))
    near_x = ORIGIN[0] + 21 * RES
    xy = np.array([center(i, j) for i, j in ij])
    path = np.array(PATH_X)
    hit = find_path_obstacle(ij, xy, path, ROBOT, CORRIDOR, TRIGGER, RES)
    assert hit["centroid"][0] == approx(near_x, abs=0.05)
    hit = find_path_obstacle(ij, xy, path, ROBOT, CORRIDOR, TRIGGER, RES,
                             skip=lambda c: c[0] < 0.5)
    assert hit["centroid"][0] > 0.8
    assert find_path_obstacle(ij, xy, path, ROBOT, CORRIDOR, TRIGGER, RES,
                              skip=lambda c: True) is None


def test_19_msg_has_sweep_count():
    from mona_interfaces.msg import PathObstacle
    assert PathObstacle().sweep_count == 0


def test_ignored_radius_inside_outside():
    ig = []
    e = add_ignored(ig, (1.0, 2.0), 0.1, 0.0)
    assert e["radius"] == approx(0.30)          # max(0.05, 0.15) + 0.15
    add_ignored(ig, (5.0, 0.0), 1.0, 0.0)       # radius 0.65
    assert is_ignored(ig, (1.2, 2.1))
    assert not is_ignored(ig, (1.4, 2.0))
    assert is_ignored(ig, (5.0, 0.6))
    assert not is_ignored([], (1.0, 2.0))


def test_ignored_expiry():
    ig = []
    add_ignored(ig, (0.0, 0.0), 0.2, 0.0)
    add_ignored(ig, (3.0, 0.0), 0.2, 20.0)
    prune_ignored(ig, 31.0, 30.0)
    assert len(ig) == 1 and not is_ignored(ig, (0.0, 0.0)) and is_ignored(ig, (3.0, 0.0))


# push entries: face mid (5, 0), robot approaches along +x (normal -x), robot ends at (5.8, 0)
def _push_entry():
    tracked = []
    e = track_push(tracked, (5.0, 0.0), (-1.0, 0.0), (5.8, 0.0), 0.2, CORRIDOR, 2.0, 0.0)
    return tracked, e


def test_push_entry_pred_is_robot_plus_tip_offset():
    tracked, e = _push_entry()
    assert e["kind"] == "push" and e["count"] == 1
    assert e["pred"] == approx((6.0, 0.0))
    assert e["mid"] == approx((5.0, 0.0))


def test_push_assoc_on_line_inside_range():
    tracked, e = _push_entry()
    for c in [(6.0, 0.0), (6.5, 0.2), (5.0 + 2.0, -0.26)]:
        got, outside = classify_tracked(tracked, c, CORRIDOR, ASSOC, 2.0)
        assert got is e and not outside


def test_push_assoc_off_line_or_range_rejected():
    tracked, _ = _push_entry()
    for c in [(6.0, 0.5), (7.1, 0.0), (4.6, 0.0)]:    # off line, past range, behind -0.3
        assert classify_tracked(tracked, c, CORRIDOR, ASSOC, 2.0) == (None, False)
    got, _ = classify_tracked(tracked, (4.75, 0.0), CORRIDOR, ASSOC, 2.0)   # -0.25 behind: ok
    assert got is not None


def test_push_never_outside_even_if_far_along_line():
    tracked, e = _push_entry()
    got, outside = classify_tracked(tracked, (6.9, 0.25), CORRIDOR, ASSOC, 2.0)
    assert got is e and outside is False


def test_second_push_increments_and_updates_pred():
    tracked, e = _push_entry()
    e2 = track_push(tracked, (6.0, 0.05), (-1.0, 0.0), (6.8, 0.05), 0.2, CORRIDOR, 2.0, 1.0)
    assert e2 is e and len(tracked) == 1 and e["count"] == 2
    assert e["pred"] == approx((7.0, 0.05))
    assert e["mid"] == approx((5.0, 0.0))


def test_push_elsewhere_gets_own_entry():
    tracked, _ = _push_entry()
    track_push(tracked, (5.0, 3.0), (-1.0, 0.0), (5.8, 3.0), 0.2, CORRIDOR, 2.0, 1.0)
    assert len(tracked) == 2


def test_sweep_and_push_counts_published_separately():
    tracked, push_e = _push_entry()
    sweep_e = track_sweep(tracked, (20.0, 0.0), (-1.0, 0.0), 0.0, ASSOC, SHIFT)
    track_sweep(tracked, (20.0, 0.0), (-1.0, 0.0), 1.0, ASSOC, SHIFT)
    assert counts_of(sweep_e) == (2, 0)
    assert counts_of(push_e) == (0, 1)
    assert counts_of(None) == (0, 0)
    # a sweep near a push entry is not merged into it
    assert track_sweep(tracked, (5.0, 0.0), (-1.0, 0.0), 2.0, ASSOC, SHIFT)["kind"] == "sweep"


def test_push_count_field_in_msg():
    from mona_interfaces.msg import PathObstacle
    assert PathObstacle().push_count == 0
