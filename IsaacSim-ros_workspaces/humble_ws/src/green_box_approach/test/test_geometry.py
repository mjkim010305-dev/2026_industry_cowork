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
    bearing_from_pixel,
    box_center_from_front,
    fallback_range_from_height,
    hsv_mask_to_blob,
    nearest_cluster,
    select_scan_span,
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


def test_bearing_from_pixel_center_is_zero():
    # px == cx -> straight ahead
    assert bearing_from_pixel(px=320, fx=500.0, cx=320.0) == pytest.approx(0.0)


def test_bearing_from_pixel_right_is_negative():
    # px > cx (right of centre) -> negative bearing (right turn convention)
    # atan2(-(400-320), 500) = atan2(-80, 500) = -0.15866 rad (hand-computed)
    got = bearing_from_pixel(px=400, fx=500.0, cx=320.0)
    assert got == pytest.approx(-0.15866, abs=1e-5)


def test_bearing_from_pixel_left_is_positive():
    # px < cx (left of centre) -> positive bearing
    # atan2(-(240-320), 500) = atan2(80, 500) = 0.15866 rad
    got = bearing_from_pixel(px=240, fx=500.0, cx=320.0)
    assert got == pytest.approx(0.15866, abs=1e-5)


def test_select_scan_span_filters_by_angle_and_range():
    # 5 rays spanning -0.2 .. 0.2 rad in 0.1 rad steps
    ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
    points = select_scan_span(
        ranges, angle_min=-0.2, angle_increment=0.1,
        bearing_lo=-0.05, bearing_hi=0.05, margin=0.0,
        range_min=0.1, range_max=10.0)
    # angles are -0.2,-0.1,0.0,0.1,0.2 -> only angle==0.0 (index 2, range 3.0)
    # falls inside [-0.05, 0.05]
    assert points == [(pytest.approx(0.0, abs=1e-9), 3.0)]


def test_select_scan_span_margin_widens_window():
    ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
    points = select_scan_span(
        ranges, angle_min=-0.2, angle_increment=0.1,
        bearing_lo=-0.05, bearing_hi=0.05, margin=0.1,
        range_min=0.1, range_max=10.0)
    # window widens to [-0.15, 0.15] -> indices 1,2,3 (angles -0.1,0.0,0.1)
    got_ranges = [r for _, r in points]
    assert got_ranges == [2.0, 3.0, 4.0]


def test_select_scan_span_drops_out_of_range_and_nonfinite():
    ranges = [0.05, float("inf"), 3.0, float("nan"), 20.0]
    points = select_scan_span(
        ranges, angle_min=0.0, angle_increment=0.0,
        bearing_lo=0.0, bearing_hi=0.0, margin=0.0,
        range_min=0.1, range_max=10.0)
    # all 5 rays share angle 0.0 (increment 0) so only range validity matters:
    # 0.05 < range_min, inf/nan invalid, 3.0 valid, 20.0 > range_max
    assert points == [(0.0, 3.0)]


def test_nearest_cluster_picks_min_range():
    points = [(0.1, 3.0), (0.0, 1.5), (-0.1, 2.0)]
    assert nearest_cluster(points) == (0.0, 1.5)


def test_nearest_cluster_empty_returns_none():
    assert nearest_cluster([]) is None


def test_fallback_range_from_height():
    # range = fy * real_height_m / pixel_h = 500 * 0.3 / 100 = 1.5 m
    got = fallback_range_from_height(pixel_h=100.0, real_height_m=0.3, fy=500.0)
    assert got == pytest.approx(1.5)


def test_fallback_range_from_height_rejects_zero_pixel_height():
    with pytest.raises(ValueError):
        fallback_range_from_height(pixel_h=0.0, real_height_m=0.3, fy=500.0)


def test_box_center_from_front_pushes_half_depth_along_ray():
    # front at (2.0, 1.0), ray unit (1, 0) (straight ahead), depth 0.4
    # -> center = (2.0 + 0.2, 1.0 + 0.0) = (2.2, 1.0)
    center = box_center_from_front(front_xy=(2.0, 1.0), ray_unit_xy=(1.0, 0.0),
                                    box_depth_m=0.4)
    assert center == pytest.approx((2.2, 1.0))


def test_box_center_from_front_diagonal_ray():
    # ray unit (cos45, sin45), depth 2.0 -> half=1.0 pushed diagonally
    ray = (math.cos(math.pi / 4), math.sin(math.pi / 4))
    center = box_center_from_front(front_xy=(0.0, 0.0), ray_unit_xy=ray,
                                    box_depth_m=2.0)
    assert center == pytest.approx((math.cos(math.pi / 4), math.sin(math.pi / 4)))
