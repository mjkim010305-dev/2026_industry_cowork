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

"""Pure geometry/vision helpers for the green box detector.

No ROS imports here on purpose (Session 11 contract) - every function is a
plain numeric/array transform so it can be unit-tested with hand-computed
expectations (see ``test/test_geometry.py``) without bringing up a node or a
simulator. ``detector_node.py`` is the only place that wraps these in
subscriptions, TF lookups and publishers.
"""

import cv2
import numpy as np


def hsv_mask_to_blob(mask, min_area):
    """Largest contour in a binary HSV mask, as a pixel bbox.

    ``mask`` is a single-channel array where nonzero means "green". Returns
    ``(x, y, w, h)`` of the largest contour's bounding box, or ``None`` if
    the mask is empty or the largest contour's area is below ``min_area``
    (rejects speckle from HSV noise rather than treating it as the box).
    """
    mask = np.asarray(mask, dtype=np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(largest)
    return (x, y, w, h)


def bearing_from_pixel(px, fx, cx):
    """Horizontal bearing (rad) of image column ``px`` under a pinhole model.

    Positive bearing is to the robot's left (REP-103: z-up, x-forward,
    y-left), so a pixel right of the principal point (``px > cx``) yields a
    negative bearing.
    """
    return float(np.arctan2(-(px - cx), fx))


def select_scan_span(ranges, angle_min, angle_increment, bearing_lo, bearing_hi,
                      margin, range_min, range_max):
    """LaserScan points whose bearing falls within ``[bearing_lo, bearing_hi]``
    (widened by ``margin`` on each side) and whose range is a valid return.

    Returns a list of ``(angle, range)`` pairs, one per matching scan index.
    """
    lo = bearing_lo - margin
    hi = bearing_hi + margin
    points = []
    for i, r in enumerate(ranges):
        if not np.isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        if lo <= angle <= hi:
            points.append((angle, float(r)))
    return points


def nearest_cluster(points):
    """Nearest-range point among ``(angle, range)`` pairs, or ``None``.

    "Cluster" here is deliberately just the closest single return: the box
    is a single rigid object, so the nearest span point is its front face
    and no further clustering is needed.
    """
    if not points:
        return None
    return min(points, key=lambda p: p[1])


def fallback_range_from_height(pixel_h, real_height_m, fy):
    """Range estimate from a known real-world height and its pixel extent.

    Pinhole similar-triangles: ``pixel_h / fy == real_height_m / range``, so
    ``range = fy * real_height_m / pixel_h``. Used when ``select_scan_span``
    returns nothing (lidar plane misses the box - see README limits).
    """
    if pixel_h <= 0:
        raise ValueError("pixel_h must be > 0")
    return fy * real_height_m / pixel_h


def box_center_from_front(front_xy, ray_unit_xy, box_depth_m):
    """Box centre from its detected front face, pushed back half a depth
    along the camera ray that found it.

    ``front_xy`` is the front-face point, ``ray_unit_xy`` a unit vector
    pointing from sensor to box (i.e. the direction to push further away
    along), ``box_depth_m`` the box's front-to-back extent.
    """
    fx, fyc = front_xy
    rx, ry = ray_unit_xy
    half = box_depth_m / 2.0
    return (fx + half * rx, fyc + half * ry)
