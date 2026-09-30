#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for green_box_approach.geometry.

Every expectation below is hand-computed from the synthetic input, not
re-derived from the function under test - a tautological test (e.g. asserting
the function against itself) would pass even if the formula were wrong.
"""

import math

import numpy as np
import pytest

from green_box_approach.geometry import (
    bbox_touches_vertical_border,
    box_center_from_front,
    confirm_window_mean,
    contiguous_cluster_centroid,
    fallback_point_from_height,
    hsv_mask_to_blob,
    is_bbox_wide_enough,
    nearest_by_range,
    rotation_matrix_from_quaternion,
    scan_points_xyz,
    select_points_in_column_window,
    transform_point,
    transform_points,
)


def test_hsv_mask_to_blob_finds_rectangle():
    mask = np.zeros((100, 200), dtype=np.uint8)
    # 40x30 rectangle at (60, 20) -> (100, 50): area = 40*30 = 1200
    mask[20:50, 60:100] = 255
    bbox = hsv_mask_to_blob(mask, min_area=100)
    assert bbox == (60, 20, 40, 30)


def test_hsv_mask_to_blob_rejects_below_min_area():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[20:25, 60:65] = 255  # 5x5 = 25 px
    assert hsv_mask_to_blob(mask, min_area=100) is None


def test_hsv_mask_to_blob_empty_mask_returns_none():
    mask = np.zeros((100, 200), dtype=np.uint8)
    assert hsv_mask_to_blob(mask, min_area=1) is None


def test_hsv_mask_to_blob_picks_largest_of_two_contours():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[10:20, 10:20] = 255      # 10x10 = 100 px, small blob
    mask[40:80, 40:120] = 255     # 40x80 = 3200 px, large blob
    bbox = hsv_mask_to_blob(mask, min_area=1)
    assert bbox == (40, 40, 80, 40)


def test_scan_points_xyz_converts_polar_to_cartesian():
    # 3 rays at angles -0.5, 0.0, 0.5 rad, all valid ranges
    ranges = [2.0, 3.0, 4.0]
    pts = scan_points_xyz(ranges, angle_min=-0.5, angle_increment=0.5,
                           range_min=0.1, range_max=10.0)
    expected = [
        (2.0 * math.cos(-0.5), 2.0 * math.sin(-0.5), 0.0),
        (3.0 * math.cos(0.0), 3.0 * math.sin(0.0), 0.0),
        (4.0 * math.cos(0.5), 4.0 * math.sin(0.5), 0.0),
    ]
    assert len(pts) == 3
    for got, want in zip(pts, expected):
        assert got == pytest.approx(want)


def test_scan_points_xyz_drops_out_of_range_and_nonfinite():
    ranges = [0.05, float("inf"), 3.0, float("nan"), 20.0]
    pts = scan_points_xyz(ranges, angle_min=0.0, angle_increment=0.0,
                           range_min=0.1, range_max=10.0)
    # only index 2 (range 3.0) is finite and inside [0.1, 10.0]; angle 0 -> (3,0,0)
    assert pts == [pytest.approx((3.0, 0.0, 0.0))]


def test_rotation_matrix_identity_quaternion():
    r = rotation_matrix_from_quaternion(0.0, 0.0, 0.0, 1.0)
    assert r == pytest.approx(np.eye(3))


def test_rotation_matrix_90deg_about_x():
    # q = (sin(45deg), 0, 0, cos(45deg)) -> +90 deg rotation about x.
    # Hand-derived: R = [[1,0,0],[0,0,-1],[0,1,0]] (y->z, z->-y).
    s = math.sqrt(0.5)
    r = rotation_matrix_from_quaternion(s, 0.0, 0.0, s)
    expected = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    assert r == pytest.approx(expected, abs=1e-9)


def test_transform_point_translation_only():
    got = transform_point((1.0, 2.0, 3.0), translation=(0.5, -1.0, 2.0), quat=(0, 0, 0, 1))
    assert got == pytest.approx((1.5, 1.0, 5.0))


def test_transform_points_rotation_and_translation():
    # Same 90deg-about-x rotation as above, plus a translation. Point
    # (1.0, 2.0, 0.0) -> R@p = (1.0, 0.0, 2.0) -> + t(0.1, 0.2, 0.5)
    # = (1.1, 0.2, 2.5).
    s = math.sqrt(0.5)
    got = transform_points([(1.0, 2.0, 0.0)], translation=(0.1, 0.2, 0.5), quat=(s, 0.0, 0.0, s))
    assert got == [pytest.approx((1.1, 0.2, 2.5))]


def test_select_points_in_column_window_rejects_behind_camera_and_wrong_column():
    """Regression for the R2 defect: the old code matched by comparing a
    yaw-only "camera bearing" against a raw scan angle, in two different
    axis conventions, and never checked whether a point was actually in
    front of the camera. Here the camera is rotated 90 degrees about x
    relative to the scan frame (an optical-frame-style mount) with a
    translation offset - exactly the case the old yaw extraction got wrong,
    since this rotation's yaw component is 0 (see
    test_rotation_matrix_90deg_about_x) even though the true rotation is
    far from identity.

    Three scan-frame points:
      A = (1.0, 2.0, 0.0)  -> camera (1.0, 0.0, 2.5): in front, in column window
      B = (1.0, -2.0, 0.0) -> camera (1.0, 0.0, -1.5): BEHIND the camera (z<=0)
      C = (5.0, 2.0, 0.0)  -> camera (5.0, 0.0, 2.5): in front, but far outside
                               the column window (wrong bbox column)
    A yaw-only, depth-blind match (old code) could not tell B and C apart
    from A by angle alone; the new full-TF + pinhole-projection path
    correctly keeps only A.
    """
    s = math.sqrt(0.5)
    quat = (s, 0.0, 0.0, s)
    translation = (0.0, 0.0, 0.5)
    scan_pts = [(1.0, 2.0, 0.0), (1.0, -2.0, 0.0), (5.0, 2.0, 0.0)]
    cam_pts = transform_points(scan_pts, translation, quat)
    assert cam_pts == [
        pytest.approx((1.0, 0.0, 2.5)),
        pytest.approx((1.0, 0.0, -1.5)),
        pytest.approx((5.0, 0.0, 2.5)),
    ]

    fx, cx = 500.0, 320.0
    # u_A = 500*1.0/2.5 + 320 = 520 -> window [500, 540] keeps only A
    kept = select_points_in_column_window(cam_pts, fx, cx, u_lo=500.0, u_hi=540.0)
    assert len(kept) == 1
    r, point = kept[0]
    assert point == pytest.approx((1.0, 0.0, 2.5))
    assert r == pytest.approx(math.sqrt(1.0 ** 2 + 2.5 ** 2))


def test_select_points_in_column_window_z_epsilon_boundary():
    # x=0 keeps u == cx regardless of z, isolating the z_epsilon boundary
    # itself: z exactly at epsilon is rejected (not "> epsilon"); just above
    # is kept.
    pts = [(0.0, 0.0, 1e-6), (0.0, 0.0, 1e-6 + 1e-9)]
    kept = select_points_in_column_window(pts, fx=500.0, cx=320.0, u_lo=0.0, u_hi=1000.0,
                                           z_epsilon=1e-6)
    assert len(kept) == 1
    assert kept[0][1] == pytest.approx((0.0, 0.0, 1e-6 + 1e-9))


def test_nearest_by_range_picks_min_range():
    ranged = [(3.0, (0.0, 0.0, 3.0)), (1.5, (0.0, 0.0, 1.5)), (2.0, (0.0, 0.0, 2.0))]
    assert nearest_by_range(ranged) == (0.0, 0.0, 1.5)


def test_nearest_by_range_empty_returns_none():
    assert nearest_by_range([]) is None


def _ranged(points):
    return [(math.sqrt(x * x + y * y + z * z), (x, y, z)) for x, y, z in points]


def test_contiguous_cluster_centroid_face_excludes_wall_behind():
    # Box face at z=1.0 sampled every 0.02 m from x=-0.1 to 0.1 (centroid
    # x=0.0), then a wall 0.5 m further back in the same columns.
    face = [(-0.1 + 0.02 * i, 0.0, 1.0) for i in range(11)]
    wall = [(0.12 + 0.02 * i, 0.0, 1.5) for i in range(5)]
    c = contiguous_cluster_centroid(_ranged(face + wall), 0.05)
    assert c == pytest.approx((0.0, 0.0, 1.0))


def test_contiguous_cluster_centroid_is_not_the_nearest_corner():
    # Face seen off-axis: nearest point is the x=0.0 end, but the centroid
    # of the face x=0.0..0.2 is x=0.1 (what the old nearest-point rule missed).
    face = [(0.02 * i, 0.0, 1.0) for i in range(11)]
    c = contiguous_cluster_centroid(_ranged(face), 0.05)
    assert c == pytest.approx((0.1, 0.0, 1.0))


def test_contiguous_cluster_centroid_stops_at_side_gap():
    # A side wall 0.1 m to the right of the face edge (gap > 0.05) is not merged.
    face = [(-0.04, 0.0, 1.0), (-0.02, 0.0, 1.0), (0.0, 0.0, 1.0), (0.02, 0.0, 1.0)]
    side = [(0.12, 0.0, 1.05), (0.14, 0.0, 1.05)]
    c = contiguous_cluster_centroid(_ranged(face + side), 0.05)
    assert c == pytest.approx((-0.01, 0.0, 1.0))


def test_contiguous_cluster_centroid_empty_returns_none():
    assert contiguous_cluster_centroid([], 0.05) is None


def test_fallback_point_from_height():
    # z = fy * real_height_m / pixel_h = 500 * 0.3 / 100 = 1.5
    # x = (u_c - cx) / fx * z = (340 - 320) / 500 * 1.5 = 0.06
    # y = (v_c - cy) / fy * z = (260 - 240) / 500 * 1.5 = 0.06
    got = fallback_point_from_height(u_c=340.0, v_c=260.0, pixel_h=100.0,
                                      real_height_m=0.3, fx=500.0, fy=500.0, cx=320.0, cy=240.0)
    assert got == pytest.approx((0.06, 0.06, 1.5))


def test_fallback_point_from_height_rejects_zero_pixel_height():
    with pytest.raises(ValueError):
        fallback_point_from_height(u_c=320.0, v_c=240.0, pixel_h=0.0, real_height_m=0.3,
                                    fx=500.0, fy=500.0, cx=320.0, cy=240.0)


def test_box_center_from_front_pushes_half_depth_along_ray():
    # front at (2.0, 1.0), camera origin at (0.0, 1.0) -> ray (1, 0), depth 0.4
    # -> center = (2.0 + 0.2, 1.0 + 0.0) = (2.2, 1.0)
    center = box_center_from_front(front_xy=(2.0, 1.0), camera_origin_xy=(0.0, 1.0),
                                    box_depth_m=0.4)
    assert center == pytest.approx((2.2, 1.0))


def test_box_center_from_front_diagonal_ray():
    # camera at origin, front at (cos45, sin45) * 3 -> unit ray (cos45, sin45),
    # depth 2.0 -> half=1.0 pushed diagonally
    d = 3.0
    front = (d * math.cos(math.pi / 4), d * math.sin(math.pi / 4))
    center = box_center_from_front(front_xy=front, camera_origin_xy=(0.0, 0.0), box_depth_m=2.0)
    expected = (front[0] + math.cos(math.pi / 4), front[1] + math.sin(math.pi / 4))
    assert center == pytest.approx(expected)


def test_box_center_from_front_rejects_degenerate_ray():
    with pytest.raises(ValueError):
        box_center_from_front(front_xy=(1.0, 1.0), camera_origin_xy=(1.0, 1.0), box_depth_m=0.3)


# --- Session 11 R4 (g6 live-run defect: occluded sliver read as a valid
# detection, resolved against the wall behind the box) -------------------

def test_is_bbox_wide_enough_rejects_narrow_blob():
    # g6-like sliver: 3 px wide, well under a 20 px threshold.
    assert is_bbox_wide_enough((100, 50, 3, 40), min_width_px=20) is False


def test_is_bbox_wide_enough_accepts_wide_blob():
    # 60 px wide, boundary-inclusive at exactly the threshold.
    assert is_bbox_wide_enough((100, 50, 20, 40), min_width_px=20) is True


def test_confirm_window_mean_tight_frames_returns_mean():
    positions = [(31.0, -2.30), (31.02, -2.28), (30.98, -2.32)]
    # mean = ((31.0+31.02+30.98)/3, (-2.30-2.28-2.32)/3) = (31.0, -2.30)
    got = confirm_window_mean(positions, radius=0.15)
    assert got == pytest.approx((31.0, -2.30))


def test_confirm_window_mean_outlier_returns_none():
    # g6-like case: two tight readings near the real box, one bad reading
    # (wall-behind-the-sliver) ~0.5 m off, mixed into the same window.
    positions = [(31.0, -2.30), (31.02, -2.28), (31.5, -2.30)]
    assert confirm_window_mean(positions, radius=0.15) is None


# --- Session 11 R7b (contact-range defect: bbox clipped by the image
# top/bottom border makes the camera-fallback height-based range wrong) ---

def test_bbox_touches_vertical_border_touching_top():
    # y=0 -> flush with the top edge.
    assert bbox_touches_vertical_border((100, 0, 40, 60), image_height=480,
                                         edge_margin_px=3) is True


def test_bbox_touches_vertical_border_touching_bottom():
    # y+h = 480 -> flush with the bottom edge (image_height=480).
    assert bbox_touches_vertical_border((100, 420, 40, 60), image_height=480,
                                         edge_margin_px=3) is True


def test_bbox_touches_vertical_border_within_margin_counts_as_touching():
    # y=2 is within edge_margin_px=3 of the top (0) - HSV/contour jitter.
    assert bbox_touches_vertical_border((100, 2, 40, 60), image_height=480,
                                         edge_margin_px=3) is True


def test_bbox_touches_vertical_border_clear_of_both_edges():
    # y=200, y+h=260, well clear of top (0) and bottom (480) with margin 3.
    assert bbox_touches_vertical_border((100, 200, 40, 60), image_height=480,
                                         edge_margin_px=3) is False
