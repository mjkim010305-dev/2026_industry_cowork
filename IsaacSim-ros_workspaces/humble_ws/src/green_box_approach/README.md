# green_box_approach

AI Festa 실기 데모(TurtleBot3 + manipulation, Nav2 Humble)용 초록 박스 검출 패키지.
**객체인식 AI(VLM/YOLO 등) 미사용** — HSV 색상 + LIDAR(`/scan`)만으로 초록 박스를 찾는다.
접근-정지 동작 자체는 이 패키지가 아니라 별도 C++ BT 노드(`custom_nav2_bt_plugins`, Lane C)가 맡는다.

## 목적

- 코스 안 움직이는 장애물을 "초록 박스"로 고정하고, `/camera/image_raw`(HSV)와 `/scan`(LIDAR)을 융합해
  박스 중심 위치(`green_box/pose`)와 검출 여부(`green_box/detected`)를 발행한다.
- Nav2 BT.CPP v3 노드(`IsGreenBoxDetectedCondition`, `ComputeGreenBoxApproachGoalAction`,
  `FinalApproachStopAction`)가 이 두 토픽을 구독해 "정면 바로 앞까지 접근 후 정지"를 수행한다.
  이 패키지는 감지만, 접근/정지 동작은 BT 노드 쪽이다(역할 분리, 계약 표 참조).

## 토픽

| 토픽 | 타입 | 방향 | 프레임 | 의미 |
|---|---|---|---|---|
| `image_topic`(param, 기본 `/camera/image_raw`) | `sensor_msgs/Image` | 구독 | camera optical | RGB8/BGR8 |
| `camera_info_topic`(param, 기본 `/camera/camera_info`) | `sensor_msgs/CameraInfo` | 구독 | camera optical | fx/fy/cx/cy |
| `scan_topic`(param, 기본 `/scan`) | `sensor_msgs/LaserScan` | 구독 | 예: `base_scan` | m, rad |
| `green_box/pose` | `geometry_msgs/PoseStamped` | 발행 | `output_frame`(param, 기본 `map`) | 박스 중심 추정(m), orientation 미사용(단위쿼터니언) |
| `green_box/detected` | `std_msgs/Bool` | 발행 | — | 이번 주기 blob+range 해소 성공 여부(매 주기 발행) |
| `green_box/debug_image` | `sensor_msgs/Image` | 발행(튜닝용) | image_topic과 동일 | HSV mask 오버레이 + bbox + range/source 텍스트 |

실제 TurtleBot3 토픽명은 이 세션에서 확인 불가(로봇 미접근) — **배포 전 반드시 실제 토픽명으로 파라미터를
덮어쓸 것** (R5).

## 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `image_topic` | `/camera/image_raw` | RGB 구독 토픽 |
| `camera_info_topic` | `/camera/camera_info` | 카메라 내부파라미터 구독 토픽 |
| `scan_topic` | `/scan` | LIDAR 구독 토픽 |
| `output_frame` | `map` | `green_box/pose`가 발행되는 프레임. `ComputeGreenBoxApproachGoalAction`의 `global_frame`과 **반드시 같은 값**이어야 함(R7) |
| `hsv_lower` / `hsv_upper` | `[40,60,40]` / `[80,255,255]` | **PLACEHOLDER** — OpenCV HSV(H 0-179) 초록 범위. 실기 박스/조명으로 재튜닝 필수(아래 절차) |
| `min_area` | `200` | HSV 마스크에서 박스로 인정할 최소 컨투어 면적(px²) |
| `morph_kernel` | `5` | 마스크 노이즈 제거용 morphological open 커널 크기(px) |
| `bearing_margin` | `0.05` | blob 각도 창을 양쪽으로 넓히는 여유(rad) |
| `range_min` / `range_max` | `0.05` / `8.0` | 유효 스캔 거리 범위(m) |
| `box_height_m` | `0.30` | **PLACEHOLDER, 나브팀 실측 필요** — 카메라 단독 폴백 경로에서만 사용 |
| `box_depth_m` | `0.30` | **PLACEHOLDER, 나브팀 실측 필요** — front face에서 박스 중심까지 미는 거리 |
| `tf_timeout` | `0.2` | TF lookup 타임아웃(초) |

## 실행

```bash
source /opt/ros/humble/setup.bash
cd IsaacSim-ros_workspaces/humble_ws
colcon build --packages-select green_box_approach
source install/setup.bash
ros2 launch green_box_approach green_box_approach.launch.py
# 파라미터 오버라이드:
ros2 launch green_box_approach green_box_approach.launch.py \
  params_file:=src/green_box_approach/config/green_box_approach_params.yaml
```

`green_box_approach_params.yaml`은 전체 기본값을 복붙한 파일이 **아니다** — 실기 배포 시 바뀔 가능성이 높은
토픽명·HSV값만 담은 오버라이드 예시다. 그 외 값은 `detector_node.py`의 노드 기본값을 그대로 쓴다(단일 소스
원칙 — launch/yaml에 값을 복제하면 노드 기본값이 바뀌어도 조용히 stale해진다).

## HSV 튜닝 절차

1. `green_box/debug_image`를 `rqt_image_view` 등으로 띄워 마스크 오버레이를 확인한다.
2. 박스가 마스크에 안 걸리면 `hsv_lower`/`hsv_upper`의 H(색상) 범위를 넓히고, 조명 반사로 다른 물체가 같이
   걸리면 S(채도)/V(명도) 하한을 올린다.
3. 노이즈 스펙클이 남으면 `morph_kernel`을 키우거나 `min_area`를 올린다.
4. 디버그 이미지 텍스트(`detected=... source=lidar|camera-fallback`)로 어느 경로가 쓰였는지 확인한다.

## Lane C(BT 노드) 연결법

- `IsGreenBoxDetectedCondition`이 `green_box/detected`(Bool)와 `green_box/pose`(PoseStamped, freshness는
  타임스탬프로 판단)를 구독해 SUCCESS/FAILURE를 결정한다.
- `ComputeGreenBoxApproachGoalAction`이 `green_box/pose`를 구독해 접근 목표(PoseStamped)를 계산한다. 이때
  `global_frame` 포트는 이 패키지의 `output_frame` 파라미터와 반드시 같은 값이어야 한다(R7).
- `FinalApproachStopAction`은 이 패키지를 구독하지 않는다 — 정지 단계는 `/scan` 전방 원뿔 최소거리만으로
  독립 동작한다(계약 근거 참조, custom_nav2_bt_plugins 쪽 문서).

## 알려진 한계

- **라이다 평면 높이 vs 박스 높이**: `/scan`은 보통 고정 높이 평면만 스캔한다. 그 평면이 박스 높이 범위를
  벗어나면(박스가 낮거나 스캔 평면이 그 위/아래) LIDAR가 박스를 못 잡는다 — 이 경우 `box_height_m`과 blob
  픽셀 높이로 거리를 추정하는 카메라 단독 폴백 경로(`source=camera-fallback`)로 자동 전환된다. 폴백은
  `box_height_m`이 실측값이 아니면 부정확하다.
- **초록색 오탐**: 코스 안에 다른 초록 물체(표지판·바닥 마킹 등)가 있으면 그 blob이 잡힐 수 있다. `hsv_lower`/
  `hsv_upper`·`min_area`는 현장 튜닝 전제이며, 이 세션은 코드 배선까지만 다룬다.
- **조명 민감도**: HSV는 조명 변화(그림자·역광 등)에 약하다. 이 패키지는 시뮬/정적 이미지로만 검증했고,
  실기 조명 재현은 이번 범위 밖이다.
- **camera->scan TF의 yaw 근사**: 두 센서의 마운트가 크게 기울어져 있으면(pitch/roll이 큰 경우) bearing 매칭에
  쓰는 yaw 근사가 부정확해질 수 있다. 일반적인 수평 마운트에서는 문제없다.
- **`box_height_m`/`box_depth_m`은 자리표시자(placeholder) 값**이다. 실측 후 파라미터를 덮어써야 정확한 거리
  추정이 나온다(R6).
