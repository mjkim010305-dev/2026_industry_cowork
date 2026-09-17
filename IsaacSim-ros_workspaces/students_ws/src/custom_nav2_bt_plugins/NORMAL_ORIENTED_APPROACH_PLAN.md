# 표면 법선 기반 접근 — 수정 방안 (적용 전 계획)

`test_depth.py` 의 아이디어(depth 표면 법선으로 접근 방향 계산)를 이 패키지에 통합하기 위한 계획.
**아직 적용 안 함 — 이 문서는 계획만.** 실제 반영 시 [PLUGIN_DEVELOPMENT_GUIDE.md](PLUGIN_DEVELOPMENT_GUIDE.md) 의 사본 기반 빌드/테스트 절차를 따른다.

## 0. 배경 (현재 상태)

- `DepthCentroidLocalizer::localize()` → `LocalizeResult{x,y,z,distance}` — **점 하나**, orientation 없음.
- `rgbd_obstacle_localizer_node` 는 그 점만 map 으로 TF 변환, `PoseStamped.orientation.w=1`(단위 회전) 로 발행.
- `ApproachObstacleNormal` 은 물체의 방향을 안 보고, **`{initial_path}` 의 2D 탄젠트**를 접근 방향(정면)으로 가정.
- 즉 지금은 "물체가 실제로 얼마나 돌아가 있는지" 를 아무 데서도 계산 안 함 — 경로 방향과 물체 실제 면이 어긋나면 접근이 삐딱해짐.
- 실제로 사용하는 camera info 값은 depth_camera_info 값이어야함.

## 1. 목적

물체가 실제로 얼마나 돌아가 있는지(표면 법선 → yaw)를 depth 로 직접 계산해서, 로봇이 **경로 방향이 아니라 물체 면 기준으로 정렬**된 상태로 정지하게 한다. 목표는 이후 로봇팔이 최소한의 각도 보정만으로 장애물을 들어올릴 수 있는 자세를 베이스가 미리 만들어주는 것.

## 2. 수정 방향

### 2-1. `test_depth.py` 자체 버그 수정 (통합 이전에 먼저)

- `msg.k.reshape((3,3))` → `np.array(msg.k).reshape((3,3))` (`array.array` 에 `.reshape` 없음)
- `pose_msg.pose.position.x = target_position[2]` (버그, `.z` 여야 함) → `position.z` 로 수정, `.x` 중복 대입 제거
- `camera_info_topic` 기본값을 이 프로젝트가 쓰는 `/front_camera_info` 로

### 2-2. 인터페이스 확장 — `MovableObstacleLocalizer`

- `LocalizeResult` (`movable_obstacle_localizer.hpp`) 에 필드 추가:
  ```cpp
  bool has_orientation {false};   // normal 추정 성공 여부
  double nx, ny, nz;              // 표면 법선, depth optical frame, 카메라 쪽을 향함
  ```
- `DepthCentroidLocalizer` 는 기존대로 `has_orientation=false` 유지 (하위호환) — normal 추정은 신규 구현에서만.
- 신규 구현은 **별도 클래스**로 (`DepthCentroidLocalizer` 를 고치지 않고 병행):
  - `include/custom_nav2_bt_plugins/normal_oriented_localizer.hpp` / `src/localizers/normal_oriented_localizer.cpp`
  - `movable_obstacle_localizer_plugins.xml` 에 `<class type="…::NormalOrientedLocalizer" .../>` 추가
  - `localizer_plugin` 파라미터로 전환 (pluginlib 교체 방식) → **기존 구현과 나란히 두고 config 로만 스위치**, 문제 생기면 파라미터 한 줄 되돌리면 즉시 롤백

### 2-3. 회전을 map 프레임으로 넘기는 절차 (`rgbd_obstacle_localizer_node.cpp`)

- **위치**: 기존과 동일 — 표면 대표점을 `PointStamped` 로 TF 변환 (평행이동 포함)
- **회전**: 법선 벡터는 **TF 의 회전 성분만** 적용해서 옮겨야 함 (이동 무시). `Vector3Stamped` 변환 또는 변환 쿼터니언을 직접 벡터에 곱하는 방식.
- 지상 로봇은 yaw 만 쓸 수 있으니 map 프레임 법선을 지면에 투영: `object_yaw = atan2(normal_map.y, normal_map.x)`.
- `orient_normals_towards_camera_location` 관례 유지 → 법선은 "표면에서 로봇 쪽" 방향, 접근은 `-normal` 방향.
- 출력 `PoseStamped.orientation` 을 이 yaw 로 채움 (지금처럼 `w=1` 고정 아님).
- **sanity check**: normal 추정이 나쁜 프레임(면이 아니라 모서리/잡동사니)에서 튈 수 있으므로, 산출된 `object_yaw` 가 경로 탄젠트와 일정 각도(예: ±60°) 이상 벗어나면 `has_orientation=false` 로 무효화 — 완전히 신뢰하지 않고 **폴백 가능하게** 설계.

### 2-4. 견고성 개선 (test_depth.py → 이 프로젝트 스타일로)

- 고정 픽셀 ROI `(860,525)-(1060,675)` → `DepthCentroidLocalizer` 처럼 비율 기반(`roi_width_ratio/height_ratio`, 중앙 정렬)으로.
- `16UC1`(mm)/`32FC1`(m) 인코딩 둘 다 처리 (지금은 float 만 가정).
- `min_points` 류 최소 포인트 검사 추가 (normal 추정이 신뢰할 만큼 점이 있는지).
- `cv2.imshow`/`cv2.waitKey` 제거 (또는 디버그 파라미터로 옵트인) — 상시 노드에 블로킹 GUI 부적절.

### 2-5. 성능/의존성

- Open3D 프레임당 비용 측정 → 목표 주기(10Hz) 못 맞추면 `pose_timeout(2.0s)` 이슈가 재발할 수 있음 (이번에 겨우 해결한 문제와 같은 종류).
- 필요시 Open3D 대신 PCA(공분산행렬 고유벡터) 로 직접 normal 계산 — 의존성 제거 + 속도 개선 가능. 결정은 실측 후.
- `package.xml`/`CMakeLists.txt` 에 `open3d`(Python) 또는 대체 구현 의존성 반영.

## 3. 병합 방향 — behavior tree 구조 변경

### 3-1. `ApproachObstacleNormal` 방향 계산 우선순위 변경

지금: 방향 = 항상 경로 탄젠트.
변경 후: **pose 에 유효한 orientation 이 있으면 그걸 우선 사용, 없거나 sanity check 실패 시 기존 경로 탄젠트로 폴백.** 즉 노드 인터페이스(포트)는 그대로 두고 내부 로직만 분기 — `custom_bt (copy).xml` 의 `<ApproachObstacleNormal .../>` 태그 자체는 안 바뀔 수 있음.

### 3-2. 트리 구조는 크게 안 바뀜, 단 팔 핸드오프가 필요하면 노드 추가

`custom_bt (copy).xml` 의 `HandleMovableObstacle` → `ApproachThenResume` 구조([custom_bt (copy).xml](../custom_bt%20(copy).xml))는 유지:

```
FollowPath {approach_path}
WaitForObstacleClearance (/obstacle/still_present)
ClearEntireCostmap ×2
FollowPath {initial_path}
```

- 로봇이 "물체 면에 정렬되어 멈췄다" 는 것과 "팔이 잡을 준비가 됐다" 는 것은 다른 신호. 팔 쪽이 `/movable_obstacle/pose` 안정 여부만으로 판단하면 노드 추가 불필요.
- 팔에게 명시적으로 "정렬 완료, 시작해" 신호가 필요하면 `FollowPath {approach_path}` 뒤, `WaitForObstacleClearance` 앞에 작은 신호용 노드(예: `SignalArmReady`, 1회 publish 후 SUCCESS) 추가.
- 표면 법선 결과의 신뢰도(sanity check 실패)에 따라 접근을 포기하고 우회할지 여부는 기존 `ApproachObstacleNormal` FAILURE → `ReactiveFallback` → `<FollowPath {path}>` 경로를 그대로 재사용 (별도 분기 불필요).

### 3-3. 파라미터 전환 지점

- `custom_nav2_params (copy).yaml` 의 `rgbd_obstacle_localizer` 블록에 `localizer_plugin: custom_nav2_bt_plugins::NormalOrientedLocalizer` 한 줄 추가/변경으로 스위치.
- 문제 생기면 그 한 줄만 `DepthCentroidLocalizer` 로 되돌리면 즉시 원복 (BT XML 은 안 건드림).

## 4. 기대 효과 (요약, 상세는 대화 기록)

- 물체가 경로와 어긋난 각도로 놓인 경우 정렬 정확도 향상 → 팔 그립 성공률 상승 기대.
- 물체가 경로에 거의 수직으로 놓인 전형적 케이스에서는 개선폭 작음(경로 탄젠트 ≈ 실제 법선).
- 근거리(standoff 부근)에서 유효, 원거리는 depth 노이즈로 신뢰도 낮음 — 마침 최종 정렬 시점과 일치해 유리.
- 평평한 면이 아닌 물체(작음/곡면)는 이득 작음.
- 계산 비용 증가 리스크 — sanity check + 폴백으로 최악의 경우에도 기존 동작(경로 탄젠트) 수준은 보장.

## 5. 검증 계획

[PLUGIN_DEVELOPMENT_GUIDE.md](PLUGIN_DEVELOPMENT_GUIDE.md) 절차 그대로 (`humble_ws_bttest` 사본, `custom_bt (copy).xml`/`custom_nav2_params (copy).yaml`):

1. `NormalOrientedLocalizer` 단독 빌드 후 합성/기록된 depth 프레임(`ros2 bag record`)으로 `localize()` 오프라인 검증 — 각도가 알려진 평면에 대해 normal yaw 오차 측정.
2. `rgbd_obstacle_localizer` 로 `/movable_obstacle/pose` 의 `orientation` 이 실제로 채워지는지, sanity check 가 나쁜 프레임을 걸러내는지 확인.
3. `/behavior_tree_log` 로 `ApproachObstacleNormal` 이 어느 방향(정상 케이스 vs 폴백)으로 갔는지 tick 단위 확인.
4. Isaac Sim 에서 물체를 각도 바꿔가며(정면/비스듬) 접근 자세 비교 — 경로 탄젠트 버전과 A/B.
5. 성능: `rgbd_obstacle_localizer` 주기가 10Hz 근처를 유지하는지, `pose_timeout(2.0s)` 트립 안 하는지 확인.

## 6. 롤백

- 파라미터 한 줄(`localizer_plugin`)로 즉시 `DepthCentroidLocalizer` 복귀 가능 (2-2 의 설계 의도).
- BT XML 변경이 필요했다면(3-2 의 신호 노드 추가) 사본에서만 작업 후 검증, 원본 반영은 기존 워크플로우 따름.

## 7. 미결 사항 (적용 전 결정 필요)

- [ ] Open3D 유지 vs PCA 직접 구현 — 성능 실측 후 결정
- [ ] sanity check 각도 임계값(예시 ±60°) 확정
- [ ] 팔에게 명시적 "준비 완료" 신호가 실제로 필요한지 (팔 쪽 구현 확인 필요)
- [ ] `NormalOrientedLocalizer` 의 최소 포인트 수 / ROI 비율 기본값
