#!/usr/bin/env python3
"""
scripts/perception_pipeline.py(v5) 안에 있던 transform_to_base_link()를 정식
패키지용으로 별도 모듈로 뽑아낸 것 — 계산 내용은 완전히 동일하다.

[TEST-ONLY · 교체 예정] 실제 카메라 장착 위치를 아직 모르기 때문에 넣어둔 placeholder
값과 로직이다. 서버 접속 후 실측하거나, tf2_ros의
lookupTransform("base_link", "camera_color_optical_frame", ...) 결과로 완전히
대체한다 — 열린 이슈: 이 변환을 Perception이 계속 코드로 수행하는 게 맞는지는
서버 연결 시점에 재확인 필요 (notes/message_spec.md 참고).
"""

# base_link 기준 x=전방/y=좌측/z=상단 방향의 placeholder 오프셋(미터).
CAMERA_OFFSET_IN_BASE_LINK = (0.15, 0.0, 0.30)


def transform_to_base_link(x_cam, y_cam, z_cam):
    """
    camera_color_optical_frame 기준 (X:오른쪽, Y:아래, Z:전방) 3D 좌표를
    base_link 관례(X:전방, Y:왼쪽, Z:위) 기준으로 바꾼다.

    두 가지를 함께 처리한다:
      1) 좌표축 방향 변경 — 카메라가 base_link에 대해 회전 없이 달려있다고
         가정한 단순화.
      2) CAMERA_OFFSET_IN_BASE_LINK 만큼 평행이동.
    """
    x_axis_fixed = z_cam
    y_axis_fixed = -x_cam
    z_axis_fixed = -y_cam

    off_x, off_y, off_z = CAMERA_OFFSET_IN_BASE_LINK
    return (
        x_axis_fixed + off_x,
        y_axis_fixed + off_y,
        z_axis_fixed + off_z,
    )
