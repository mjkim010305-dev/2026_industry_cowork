#!/usr/bin/env python3
"""
8단계: ROI 안의 depth 픽셀들을 3D 점들의 모음(point cloud)으로 "분리"하고,
그 점들을 감싸는 axis-aligned bounding box(AABB)로 물체의 크기(width, depth, height)를
계산하는 함수.

[전체 흐름 안에서의 위치]
지금까지(3~7단계)는 ROI의 중심 픽셀 1개 + depth 중앙값만으로 물체의 "위치"(x,y,z)만
계산했다. 이번 단계에서는 ROI 전체 픽셀(중심 1개가 아니라 여러 개)을 각각
pixel_to_3d()로 3D 점으로 바꿔서 "점 구름(point cloud)"을 만들고, 그 점들이 카메라
기준 X(좌우)/Y(상하)/Z(전후) 방향으로 얼마나 퍼져 있는지를 재서 물체의 "크기"를 추정한다.

[용어] AABB(Axis-Aligned Bounding Box) = 물체를 회전 없이 좌표축에 딱 맞춰 감싸는
직육면체. 물체가 카메라를 향해 기울어져 있어도 무시하고 "가장 넓게 뻗은 범위"만 재는
가장 단순한 크기 추정 방법이다 — orientation(자세)을 아직 다루지 않는 지금 단계에 맞다.

[Phase 1 가짜 데이터의 한계 — 미리 알아둘 것]
fake_d555_publisher.py가 만드는 가짜 depth 이미지는 ROI 안 전체를 "케이스의 depth 값
하나"로 평평하게 채운다(실제 물체 표면처럼 깊이가 미세하게 변하지 않는다). 그래서 이
함수로 계산한 depth(전후 방향 크기)는 항상 0에 가깝게 나온다 — 이건 코드 버그가
아니라 가짜 데이터가 "평평한 판자"를 흉내내기 때문이다. width/height(좌우/상하 크기)는
ROI의 가로/세로 픽셀 크기가 실제 그 depth에서 몇 미터에 해당하는지를 정확히 반영하므로
의미 있는 값이 나온다. 실제 D555를 쓰면 물체 표면의 실제 굴곡이 depth에 담기므로 이
한계는 사라진다.

[코드 사용 범위 표시]
  [PROD]                 : 서버 접속 후 진짜 데이터로도 그대로 쓰는 부분
  [TEST-ONLY · 교체 예정] : 자리는 그대로 남고 값/내용만 나중에 실제 값으로 바뀌는 부분
  [TEST-ONLY · 삭제 예정] : 통째로 사라지고, 필요하면 완전히 새 코드로 다시 작성되는 부분
"""

import json
import pathlib

import numpy as np


# [PROD] ROI(x,y,w,h) 안의 유효한(0보다 큰) depth 픽셀들을 전부 3D 점으로 바꾼다.
# pixel_to_3d()를 픽셀마다 하나씩 부르는 대신, numpy로 한 번에 계산한다(속도).
def roi_to_point_cloud(depth_img, roi, fx, fy, cx, cy):
    """
    depth_img: (H, W) numpy 배열, 단위 m
    roi: (x, y, w, h) — 픽셀 단위 사각형
    -> (N, 3) numpy 배열. 각 행이 카메라 기준 3D 점 (X, Y, Z). N=0이면 유효한 점이 없음.
    """
    x, y, w, h = roi
    depth_roi = depth_img[y:y + h, x:x + w]

    vs, us = np.where(depth_roi > 0)  # ROI 내부 기준 (row=v, col=u) 인덱스
    if us.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    depths = depth_roi[vs, us]
    U = (us + x).astype(np.float32)
    V = (vs + y).astype(np.float32)

    X = (U - cx) * depths / fx
    Y = (V - cy) * depths / fy
    Z = depths
    return np.stack([X, Y, Z], axis=1)


# [PROD] 점 구름을 감싸는 AABB로 물체 크기를 계산한다.
# 반환값의 w/d/h는 카메라 프레임 기준 X/Z/Y 축 퍼짐이지만, base_link로 좌표를
# 옮길 때 회전이 아니라 "축 이름만 바꾸는" 변환(perception_pipeline.py의
# transform_to_base_link 참고)을 쓰므로, 이 크기(폭/깊이/높이) 값 자체는 프레임이
# 바뀌어도 그대로 재사용할 수 있다.
def compute_aabb(points):
    """
    points: (N, 3) numpy 배열, (X, Y, Z) 카메라 기준.
    -> {"w": 폭(X 방향), "d": 깊이(Z 방향, 전후), "h": 높이(Y 방향), "min": (x,y,z), "max": (x,y,z)}
       점이 하나도 없으면 None.
    """
    if points.shape[0] == 0:
        return None

    mins = points.min(axis=0)
    maxs = points.max(axis=0)

    return {
        "w": float(maxs[0] - mins[0]),   # 좌우(X) 방향 폭
        "d": float(maxs[2] - mins[2]),   # 전후(Z) 방향 깊이 — Phase 1 가짜 데이터에서는 항상 ~0
        "h": float(maxs[1] - mins[1]),   # 상하(Y) 방향 높이
        "min": tuple(float(v) for v in mins),
        "max": tuple(float(v) for v in maxs),
    }


# [TEST-ONLY · 삭제 예정] fake_input.json을 읽는 테스트 전용 헬퍼.
def _load_test_cases():
    workspace_dir = pathlib.Path(__file__).resolve().parent.parent
    data_path = workspace_dir / "sample_data" / "fake_input.json"
    with open(data_path, "r") as f:
        data = json.load(f)
    return data["camera_intrinsics"], data["test_cases"]


# [TEST-ONLY · 삭제 예정] roi_to_point_cloud()/compute_aabb()가 잘 동작하는지 확인하는
# 테스트 코드. 실제 파이프라인에서는 perception_pipeline.py가 이 두 함수를 호출한다.
def main():
    intr, test_cases = _load_test_cases()
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    IMG_W, IMG_H, BG_DEPTH = 640, 480, 2.0

    for case in test_cases:
        bbox = case["bbox"]
        x, y, w, h = bbox["x"], bbox["y"], bbox["w"], bbox["h"]
        depth = case["depth"]

        depth_img = np.full((IMG_H, IMG_W), BG_DEPTH, dtype=np.float32)
        depth_img[y:y + h, x:x + w] = depth

        points = roi_to_point_cloud(depth_img, (x, y, w, h), fx, fy, cx, cy)
        aabb = compute_aabb(points)

        # AABB는 "픽셀 중심점들의 최소~최대"라서, bbox 폭(w)이 아니라
        # 픽셀 간격(w-1)만큼만 퍼진다(맨 끝 픽셀 중심부터 반대쪽 끝 픽셀 중심까지) —
        # 그래서 기대값도 w가 아니라 (w-1)로 계산한다(픽셀 1개 차이, 버그 아님).
        expected_w = (w - 1) * depth / fx
        expected_h = (h - 1) * depth / fy
        ok = (
            abs(aabb["w"] - expected_w) < 1e-3
            and abs(aabb["h"] - expected_h) < 1e-3
        )
        status = "OK" if ok else "FAIL"
        print(
            f"[{case['name']:6s}] 점 개수={points.shape[0]:>4}  "
            f"w={aabb['w']:.3f}(기대 {expected_w:.3f})  "
            f"d={aabb['d']:.3f}(가짜 데이터 한계로 항상 ~0)  "
            f"h={aabb['h']:.3f}(기대 {expected_h:.3f})  -> {status}"
        )


if __name__ == "__main__":
    main()
