#!/usr/bin/env python3
"""
3단계: 픽셀(u, v) + depth 값을 카메라 기준 3D 좌표 (X, Y, Z)로 바꾸는 함수.

이 파일은 5단계 재작업 v4(objects 배열 구조 전환)와 무관하게 변경 없음 —
계산 공식 자체는 물체가 몇 개든 그대로 재사용된다 (perception_pipeline.py가
물체마다 이 함수를 한 번씩 호출한다).

공식 (핀홀 카메라 모델):
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth(u, v)

[코드 사용 범위 표시]
  [PROD]      : 서버 접속 후 진짜 데이터로도 그대로 쓰는 부분
  [TEST-ONLY] : Phase 1 가짜 데이터 검증용으로만 쓰고, 나중에 지우거나 바꾸는 부분
"""

import json
import pathlib


# [PROD] 값이 어디서 왔는지(가짜/진짜)와 무관하게 그대로 재사용된다.
def pixel_to_3d(u, v, depth, fx, fy, cx, cy):
    """픽셀 좌표(u, v)와 depth 값을 카메라 기준 3D 좌표 (X, Y, Z)로 변환한다."""
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth
    return X, Y, Z


# [TEST-ONLY] 서버 접속 후에는 이 함수 대신, 실제 CameraInfo topic을 구독해서
# fx, fy, cx, cy를 읽어오는 코드로 바뀐다. fake_input.json은 더 이상 쓰지 않는다.
def load_fake_input():
    """sample_data/fake_input.json 을 읽어서 카메라 파라미터 + 테스트 케이스를 돌려준다."""
    workspace_dir = pathlib.Path(__file__).resolve().parent.parent
    data_path = workspace_dir / "sample_data" / "fake_input.json"
    with open(data_path, "r") as f:
        return json.load(f)


# [TEST-ONLY] pixel_to_3d()가 잘 동작하는지 확인하는 테스트 코드일 뿐이다.
# 실제 파이프라인에서는 이 main()이 아니라 perception_pipeline.py가 pixel_to_3d()를 호출한다.
def main():
    data = load_fake_input()
    intr = data["camera_intrinsics"]
    fx, fy, cx, cy = intr["fx"], intr["fy"], intr["cx"], intr["cy"]

    print(f"camera intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}\n")

    for case in data["test_cases"]:
        X, Y, Z = pixel_to_3d(case["u"], case["v"], case["depth"], fx, fy, cx, cy)
        print(
            f"[{case['name']:6s}] u={case['u']:>4} v={case['v']:>4} "
            f"depth={case['depth']:.2f}  ->  X={X:+.3f} Y={Y:+.3f} Z={Z:.3f}"
        )


if __name__ == "__main__":
    main()
