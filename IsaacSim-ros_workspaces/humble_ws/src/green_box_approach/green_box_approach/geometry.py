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

Session 11 R2 rewrite: the camera<->lidar association used to (a) treat the
camera frame as a body-convention frame (x-forward) when a real camera image
frame is an OPTICAL frame per REP-103 (x right, y down, z forward), and (b)
carry only the yaw of the camera->scan TF, dropping the translation
(parallax) between the two sensors. Both defects are fixed by working
entirely in 3D with the *full* TF (rotation + translation): lidar returns
become 3D points, get transformed into the camera's optical frame with a
full rotation matrix + translation, and are matched against the detected
blob by projecting them with the pinhole model and checking the resulting
pixel column - never by comparing a "camera bearing" against a "scan angle"
in two frames that generally do not share an axis.

Session 11 R4 (g6 live-run defect): a partially occluded box can still pass
the R2 fusion above with a plausible-looking but wrong position, because a
narrow visible sliver still projects to *some* pixel column and *some*
lidar return under that column - just the wrong object (a wall edge behind
the box, in g6). ``is_bbox_wide_enough`` and ``confirm_window_mean`` add two
independent guards against that: reject blobs too narrow to be a real box
view, and only trust a position once several consecutive frames agree on it.
"""

import math

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


def scan_points_xyz(ranges, angle_min, angle_increment, range_min, range_max):
    """Valid ``LaserScan`` returns as 3D points ``(x, y, 0.0)`` in the scan
    frame (planar lidar - no elevation, so z is always 0). Invalid returns
    (non-finite, or outside ``[range_min, range_max]``) are dropped, same
    validity rule a ``LaserScan`` consumer is expected to apply.
    """
    points = []
    for i, r in enumerate(ranges):
        if not np.isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        points.append((r * math.cos(angle), r * math.sin(angle), 0.0))
    return points


def rotation_matrix_from_quaternion(qx, qy, qz, qw):
    """Standard quaternion -> 3x3 rotation matrix (full 3D, not a yaw-only
    approximation). This is what lets a camera mount with pitch/roll (e.g.
    an optical frame, which is a 90 degree-class rotation away from a body
    frame) be handled correctly - the old code extracted only the yaw of the
    camera<->scan TF, which is wrong whenever the two frames' z-axes are not
    parallel (exactly the optical-frame case).
    """
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def transform_points(points, translation, quat):
    """Apply a full TF (rotation + translation) to a list of 3D points:
    ``p_out = R(quat) @ p_in + translation``. ``translation`` is ``(x, y,
    z)``, ``quat`` is ``(qx, qy, qz, qw)``. Used for both scan->camera and
    camera->output_frame - the parallax between sensors (the ``+
    translation`` the old code dropped) matters most at the 0.4-1.5 m
    approach range this detector operates at.
    """
    r = rotation_matrix_from_quaternion(*quat)
    t = np.array(translation)
    return [tuple(r @ np.array(p) + t) for p in points]


def transform_point(point, translation, quat):
    """Single-point convenience wrapper around :func:`transform_points`."""
    return transform_points([point], translation, quat)[0]


def select_points_in_column_window(points_cam, fx, cx, u_lo, u_hi, z_epsilon=1e-6):
    """Camera-frame points that are (a) in front of the camera
    (``z > z_epsilon``, REP-103 optical z-forward - rejects returns that are
    physically behind the camera, which a bearing-only match could never
    detect) and (b) project, under the pinhole model
    (``u = fx * x / z + cx``), inside the pixel column window
    ``[u_lo, u_hi]`` (typically the blob bbox columns widened by a margin).

    Returns a list of ``(range_from_camera, point_xyz)`` pairs for the
    points kept, ``range_from_camera`` being the euclidean distance from the
    camera origin (used to pick the box's front face - see
    :func:`nearest_by_range`).
    """
    kept = []
    for x, y, z in points_cam:
        if z <= z_epsilon:
            continue
        u = fx * x / z + cx
        if u_lo <= u <= u_hi:
            kept.append((math.sqrt(x * x + y * y + z * z), (x, y, z)))
    return kept


def nearest_by_range(ranged_points):
    """Closest point among ``(range, point_xyz)`` pairs, or ``None``.

    The box is a single rigid object, so the nearest kept point is its front
    face and no further clustering is needed (same reasoning as the old
    bearing-based ``nearest_cluster``, now applied to 3D range).
    """
    if not ranged_points:
        return None
    return min(ranged_points, key=lambda p: p[0])[1]


def fallback_point_from_height(u_c, v_c, pixel_h, real_height_m, fx, fy, cx, cy):
    """3D point (camera optical frame) from a known real-world height and
    its pixel extent, used when the lidar path keeps no points (lidar plane
    misses the box - see README limits).

    Pinhole similar-triangles gives the depth: ``pixel_h / fy ==
    real_height_m / z``, so ``z = fy * real_height_m / pixel_h``. The blob
    centre pixel ``(u_c, v_c)`` then back-projects to
    ``x = (u_c - cx) / fx * z``, ``y = (v_c - cy) / fy * z``.
    """
    if pixel_h <= 0:
        raise ValueError("pixel_h must be > 0")
    z = fy * real_height_m / pixel_h
    x = (u_c - cx) / fx * z
    y = (v_c - cy) / fy * z
    return (x, y, z)


def is_bbox_wide_enough(bbox, min_width_px):
    """Reject a blob whose bbox is narrower than ``min_width_px``.

    Session 11 g6: the box was partly hidden behind a wall edge and showed
    up as a strip only a few pixels wide; the old code accepted it as a
    normal detection and the lidar column under that sliver was the wall
    behind the box, not the box itself. A real box a few tenths of a metre
    wide, at the ~3 m range where this detector first picks it up, spans
    tens of pixels (``fx * width_m / range_m``, e.g. ~40 px for a 0.2 m box
    at 3 m with fx~634) - a few-pixel strip is an order of magnitude below
    that, so it is a wall/occlusion edge artifact, not a genuinely distant
    or small box.
    """
    _, _, w, _ = bbox
    return w >= min_width_px


def confirm_window_mean(positions, radius):
    """Mean of ``positions`` (list of ``(x, y)``) if they all agree, else
    ``None``.

    "Agree" means every point lies within ``radius`` metres of the mean of
    the window - a single stale/incorrect frame (e.g. g6's wall-behind-strip
    reading, momentarily mixed in with otherwise-good frames) pulls the mean
    away from the rest and gets caught here. Length/recency (which frames
    are actually in the window, and that they were all consecutive accepted
    detections) is the caller's responsibility - this function only judges
    spatial agreement of whatever list it is given.
    """
    if not positions:
        return None
    mean_x = sum(p[0] for p in positions) / len(positions)
    mean_y = sum(p[1] for p in positions) / len(positions)
    for x, y in positions:
        if math.hypot(x - mean_x, y - mean_y) > radius:
            return None
    return (mean_x, mean_y)


def box_center_from_front(front_xy, camera_origin_xy, box_depth_m):
    """Box centre from its detected front face, pushed back half a depth
    along the horizontal (output-frame xy) direction from the camera origin
    to the front point.

    ``front_xy`` and ``camera_origin_xy`` are both in ``output_frame``.
    Raises ``ValueError`` if they coincide (degenerate ray - would require
    guessing a push direction).
    """
    dx = front_xy[0] - camera_origin_xy[0]
    dy = front_xy[1] - camera_origin_xy[1]
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("front point coincides with camera origin (degenerate ray)")
    half = box_depth_m / 2.0
    return (front_xy[0] + half * dx / norm, front_xy[1] + half * dy / norm)
