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
| `camera_frame` | `""` (빈 문자열) | 라이다↔카메라 기하 융합에 쓰는 카메라 프레임. 빈 문자열이면 이미지 메시지의 `header.frame_id`를 그대로 쓴다. **반드시 OPTICAL 프레임**(REP-103: x=오른쪽, y=아래, z=전방)이어야 함 — 핀홀 투영이 이 규약을 전제로 한다. 이 시뮬레이션의 이미지 header 프레임은 OPTICAL이 아닌 항등(identity) 프레임이므로, 실행 시 `camera_frame:=camera_optical`(실측 정적 TF `base_link->camera_optical` 존재)로 오버라이드해야 한다 |
| `output_frame` | `map` | `green_box/pose`가 발행되는 프레임. `ComputeGreenBoxApproachGoalAction`의 `global_frame`과 **반드시 같은 값**이어야 함(R7) |
| `hsv_lower` / `hsv_upper` | `[40,60,40]` / `[80,255,255]` | **PLACEHOLDER** — OpenCV HSV(H 0-179) 초록 범위. 실기 박스/조명으로 재튜닝 필수(아래 절차) |
| `min_area` | `200` | HSV 마스크에서 박스로 인정할 최소 컨투어 면적(px²) |
| `morph_kernel` | `5` | 마스크 노이즈 제거용 morphological open 커널 크기(px) |
| `bearing_margin_px` | `20.0` | blob bbox의 열(column) 창을 양쪽으로 넓히는 여유(px). Session 11 R2부터 라디안이 아니라 픽셀 단위 — 핀홀 투영으로 라이다 점을 카메라 픽셀 열에 직접 매칭하기 때문 |
| `range_min` / `range_max` | `0.05` / `8.0` | 유효 스캔 거리 범위(m) |
| `box_height_m` | `0.30` | **PLACEHOLDER, 나브팀 실측 필요** — 카메라 단독 폴백 경로에서만 사용 |
| `box_depth_m` | `0.30` | **PLACEHOLDER, 나브팀 실측 필요** — front face에서 박스 중심까지 미는 거리 |
| `tf_timeout` | `0.2` | TF lookup 타임아웃(초) |
| `min_width_px` | `20` | (R4) bbox 폭이 이 값(px) 미만이면 거부 — 이번 프레임은 미검출로 취급. 실측 3 m 거리·fx~634에서 실제 박스(폭 수십 cm)는 수십 px를 채우므로(예: 0.2 m 폭이면 ~42 px), `20`은 그 절반 이하로 가려진 조각(g6: 벽 모서리에 가려 몇 px 띠만 보임 → 그 열의 라이다 점은 벽이었음)을 걸러내면서도 부분 가림에 여유를 둔다 |
| `confirm_frames` | `3` | (R4) 이 수만큼 **연속**으로 검출(폭 통과 + 거리 해소)이 성공하고 서로 `confirm_radius` 이내에 모여야 `green_box/detected=true`를 발행한다. 거부/미검출 프레임이 하나라도 끼면 창이 초기화된다 |
| `confirm_radius` | `0.15` | (R4) `confirm_frames`개 프레임의 해소 위치가 서로의 평균에서 이 반경(m) 안에 있어야 "확정"으로 본다. 확정 시 발행 위치는 그 프레임들의 평균 |
| `edge_margin_px` | `3` | (R7b) bbox가 이미지 위/아래 경계에서 이 값(px) 이내면 "닿았다"로 본다 — `min_area`처럼 HSV/컨투어 노이즈로 bbox가 진짜 경계에서 1~2 px 못 미치는 경우까지 잡기 위한 여유. 카메라 폴백 경로에서만 쓰임(아래 "알려진 한계") |

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
4. 디버그 이미지 텍스트(`detected=... source=lidar|camera-fallback reason=...`)로 어느 경로가 쓰였는지, R4/R7b 가드가
   이번 프레임을 어떻게 판단했는지(`narrow`/`no-range`/`clipped`/`confirming k/N`/`unstable`/`confirmed`) 확인한다.

## Lane C(BT 노드) 연결법

- `IsGreenBoxDetectedCondition`이 `green_box/detected`(Bool)와 `green_box/pose`(PoseStamped, freshness는
  타임스탬프로 판단)를 구독해 SUCCESS/FAILURE를 결정한다.
- `ComputeGreenBoxApproachGoalAction`이 `green_box/pose`를 구독해 접근 목표(PoseStamped)를 계산한다. 이때
  `global_frame` 포트는 이 패키지의 `output_frame` 파라미터와 반드시 같은 값이어야 한다(R7).
- `FinalApproachStopAction`도 `green_box/pose`를 구독한다 — 시작 직후 받은 첫 포즈를 고정해 박스 쪽으로 조향하고,
  로봇→박스 중심 거리가 `stop_center_distance` 이하이거나 `/scan` 전방 원뿔 최소거리가 `stop_distance` 이하이면
  정지한다(둘 중 먼저 걸리는 쪽). 라이다 전방이 팔에 가려 무반사(-1)여도 거리 기준 정지가 동작한다(g8).

## 알려진 한계

- **라이다 평면 높이 vs 박스 높이**: `/scan`은 보통 고정 높이 평면만 스캔한다. 그 평면이 박스 높이 범위를
  벗어나면(박스가 낮거나 스캔 평면이 그 위/아래) LIDAR가 박스를 못 잡는다 — 이 경우 `box_height_m`과 blob
  픽셀 높이로 거리를 추정하는 카메라 단독 폴백 경로(`source=camera-fallback`)로 자동 전환된다. 폴백은
  `box_height_m`이 실측값이 아니면 부정확하다.
- **초록색 오탐**: 코스 안에 다른 초록 물체(표지판·바닥 마킹 등)가 있으면 그 blob이 잡힐 수 있다. `hsv_lower`/
  `hsv_upper`·`min_area`는 현장 튜닝 전제이며, 이 세션은 코드 배선까지만 다룬다.
- **조명 민감도**: HSV는 조명 변화(그림자·역광 등)에 약하다. 이 패키지는 시뮬/정적 이미지로만 검증했고,
  실기 조명 재현은 이번 범위 밖이다.
- **`box_height_m`/`box_depth_m`은 자리표시자(placeholder) 값**이다. 실측 후 파라미터를 덮어써야 정확한 거리
  추정이 나온다(R6).
- **R4 확정 지연(latency)**: `green_box/detected`가 `true`가 되려면 `confirm_frames`(기본 3)개의 이미지 주기 동안
  연속으로 검출이 성공하고 서로 `confirm_radius` 이내여야 한다 — 즉 박스가 처음 시야에 들어온 뒤 최소
  `confirm_frames`개 이미지 주기(카메라 프레임레이트에 반비례, 예: 10 Hz면 최소 ~0.2-0.3 s)가 지나야 `detected=true`가
  뜬다. 도중에 한 프레임이라도 거부(`narrow`/`no-range`)되면 창이 초기화돼 다시 `confirm_frames`개를 채워야 한다.
  g6처럼 순간적으로 가려지는 상황이 반복되면 지연이 더 늘 수 있다.
- **접촉 거리에서는 어느 경로도 박스 거리를 잴 수 없다(R7b)**: 로봇이 박스에 거의 닿을 만큼 가까워지면 LIDAR
  전방 빔이 (TurtleBot3 manipulator의 팔에 가려지거나 박스 상단을 넘어가) 아무것도 잡지 못하고(`-1`/무효), 동시에
  카메라 이미지에서는 박스가 위/아래 경계를 넘어가 bbox 높이가 잘린다. 카메라 폴백은 이 잘린 픽셀 높이로
  거리를 역산하므로 실제보다 먼 값을 그대로 발행할 수 있다 — 그래서 이 경우는 값을 내지 않고 거부한다
  (`reason=clipped`, `edge_margin_px` 참조). **즉 두 경로 모두 접촉 거리에서는 멈춘다.** BT 쪽 최종 접근은 이
  구간에서 새 검출을 기다리지 않고, 그 전에 확정된(`confirmed`) 포즈를 고정해 쓴 채 `FinalApproachStopAction`의
  거리 기준 정지(`stop_center_distance`)와 `/scan` 전방 원뿔 정지로 넘어가는 것을 전제로 한다.
  `stop_center_distance` 기본값 0.55 m는 약 0.3 m 박스 기준 자리표시값이다 — 실제 박스 크기에 맞춰 설정해야 한다.

- **예제 트리 = final.xml 한 구간**: `green_box_approach_example.xml`은 `final.xml`의 주행 한 구간(1 Hz 재계획,
  `RecoveryNode` 최대 6회, 박스 접근·정지 후 Place/Push/Pick, `WaitForObstacleClearance` 10초, `{initial_path}` 복귀)을
  목표 하나에 대해 그대로 실행한다(왕복 루프·시작 시 Pick은 제외). 접근 시퀀스가 실패하면 `ReactiveFallback`이 다음 틱에
  다시 시도하고, 그 실패가 주행 실패로 번지면 6회 recovery 후 목표 자체가 실패한다. 단, 복귀 주행이 실패했을 때 박스가
  여전히 보이면 다시 박스로 접근하는 반복에는 별도 상한이 없다.

## use_sim_time 일관성 전제조건

`green_box/pose`의 헤더 타임스탬프는 **그 포즈를 만든 이미지 메시지의 스탬프**를 그대로 쓴다(카메라→
`output_frame` TF도 같은 스탬프로 조회한다). 이 값이 의미가 있으려면 이미지·TF·`green_box/pose`를 구독하는
Nav2 쪽(`use_sim_time` 여부와 시계)이 **모두 같은 시계**를 기준으로 해야 한다 — 시뮬레이션이면 노드 전부
`use_sim_time:=true`로 `/clock`을 따라야 하고, 실기면 전부 wall clock이어야 한다. 한쪽만 sim time이면 TF
lookup이 과거/미래 시각을 조회하게 되어 stamp-then-latest 폴백(`_lookup_transform`)이 계속 latest로만 빠지거나
아예 실패한다.
