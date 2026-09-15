#!/usr/bin/env python3
"""
scripts/pixel_to_3d.py(3단계, [PROD])를 정식 패키지 안으로 그대로 옮긴 것.
계산 공식은 완전히 동일하다 — 이 파일이 "진짜" 실행 위치이고, scripts/ 쪽은
Phase 1 동안 혼자 테스트해보기 위한 사본으로 남는다.
"""


def pixel_to_3d(u, v, depth, fx, fy, cx, cy):
    """픽셀 좌표(u, v)와 depth 값을 카메라 기준 3D 좌표 (X, Y, Z)로 변환한다."""
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
    return X, Y, Z
