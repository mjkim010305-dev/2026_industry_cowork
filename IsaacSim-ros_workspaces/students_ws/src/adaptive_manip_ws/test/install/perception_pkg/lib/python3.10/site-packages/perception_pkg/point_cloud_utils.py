#!/usr/bin/env python3
"""
scripts/point_cloud_utils.py(8단계, [PROD])를 정식 패키지 안으로 그대로 옮긴 것.
계산 내용과 가짜 데이터의 한계(깊이(d) 방향은 항상 ~0)는 scripts/ 쪽 원본의
설명과 동일하다.
"""

import numpy as np


def roi_to_point_cloud(depth_img, roi, fx, fy, cx, cy):
    """
    depth_img: (H, W) numpy 배열, 단위 m
    roi: (x, y, w, h)
    -> (N, 3) numpy 배열, 각 행이 카메라 기준 3D 점 (X, Y, Z)
    """
    x, y, w, h = roi
    depth_roi = depth_img[y:y + h, x:x + w]

    vs, us = np.where(depth_roi > 0)
    if us.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    depths = depth_roi[vs, us]
    U = (us + x).astype(np.float32)
    V = (vs + y).astype(np.float32)

    X = (U - cx) * depths / fx
    Y = (V - cy) * depths / fy
    Z = depths
    return np.stack([X, Y, Z], axis=1)


def compute_aabb(points):
    """
    points: (N, 3) numpy 배열, (X, Y, Z) 카메라 기준.
    -> {"w":..., "d":..., "h":..., "min":(x,y,z), "max":(x,y,z)} 또는 점이 없으면 None.
    """
    if points.shape[0] == 0:
        return None

    mins = points.min(axis=0)
    maxs = points.max(axis=0)

    return {
        "w": float(maxs[0] - mins[0]),
        "d": float(maxs[2] - mins[2]),
        "h": float(maxs[1] - mins[1]),
        "min": tuple(float(v) for v in mins),
        "max": tuple(float(v) for v in maxs),
    }
