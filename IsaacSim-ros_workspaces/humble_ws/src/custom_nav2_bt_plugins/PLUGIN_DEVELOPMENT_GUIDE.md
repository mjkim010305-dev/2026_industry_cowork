# custom_nav2_bt_plugins 빌드/테스트 런북

이 파일을 멘션하면 아래 절차대로 이 패키지를 빌드·테스트한다.
코드 작성은 이 디렉터리에서 대화하며 진행하고, 빌드/테스트가 필요할 때만 이 런북을 사용.

## 환경 (2026-09-01 확인)

- ROS `humble` / colcon `/usr/bin/colcon` / Nav2 `/opt/ros/humble` 바이너리 설치
- **BT.CPP 는 `behaviortree_cpp_v3` 만 설치됨** (v4 `behaviortree_cpp` 없음)
- 현재 `package.xml`·`CMakeLists.txt` 는 `behaviortree_cpp`(v4) 의존 → 수정 전에는 빌드 실패가 정상

## 원칙

- 원본 `humble_ws/{build,install,log}` 는 건드리지 않는다.
- 빌드·테스트는 복사본 워크스페이스 `humble_ws_bttest` 에서만 한다.
- 매 빌드는 새 셸에서 `source /opt/ros/humble/setup.bash` 만 (원본 install 은 source 안 함).
- **원본 `custom_bt.xml` / `custom_nav2_params.yaml` 은 절대 수정하지 않는다.** (내일 작업에 영향)
  BT 트리·파라미터 테스트는 아래 사본에서만 편집·실행한다:
  - `custom_bt (copy).xml`
  - `custom_nav2_params (copy).yaml`

## 1. 복사본 워크스페이스 준비 (없으면 생성, 있으면 src 동기화)

```bash
cd /home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces
rsync -a --delete \
  --exclude build/ --exclude install/ --exclude log/ \
  --exclude 'custom_bt (copy).xml' --exclude 'custom_nav2_params (copy).yaml' \
  humble_ws/src/ humble_ws_bttest/src/
```

## 1-1. 테스트용 사본 생성 (사본이 없을 때만; 있으면 유지)

```bash
cd /home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces/humble_ws_bttest/src
[ -f 'custom_bt (copy).xml' ]            || cp custom_bt.xml            'custom_bt (copy).xml'
[ -f 'custom_nav2_params (copy).yaml' ]  || cp custom_nav2_params.yaml  'custom_nav2_params (copy).yaml'
```

이후 BT 노드 추가·`plugin_lib_names` 등록 등 모든 편집은 이 두 사본에만 한다.

## 2. colcon 환경 확인

```bash
cd /home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces/humble_ws_bttest
source /opt/ros/humble/setup.bash
colcon list | grep custom_nav2_bt_plugins
```

## 3. 이 패키지 빌드

```bash
colcon build --packages-select custom_nav2_bt_plugins \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

판정:

- `find_package(behaviortree_cpp ...)` 실패 → 의존성을 `behaviortree_cpp_v3` 로 아직 안 고친 상태.
- 빌드 성공 → colcon 환경 정상.
- `nav2_behavior_tree` 못 찾음 → `ros2 pkg prefix nav2_behavior_tree` 로 설치 확인.

## 4. 산출물 / 로딩 확인

```bash
source install/setup.bash
ls install/custom_nav2_bt_plugins/lib/          # .so 생성 확인
```

BT 팩토리 로딩 스모크 테스트가 있으면 여기서 실행
(`factory.registerFromPlugin("install/custom_nav2_bt_plugins/lib/lib<타깃>.so")` → 트리 XML 파싱).

## 4-1. Nav2 로 로딩 테스트 (사본 params/xml 사용)

`custom_nav2_params (copy).yaml` 의 `bt_navigator > plugin_lib_names` 에 빌드된 타깃명을 추가하고,
`custom_bt (copy).xml` 에 커스텀 노드를 배치한 뒤 실행한다. **원본 파일은 지정하지 않는다.**

```bash
cd /home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces/humble_ws_bttest
source /opt/ros/humble/setup.bash
source install/setup.bash

P="$PWD/src/custom_nav2_params (copy).yaml"
B="$PWD/src/custom_bt (copy).xml"

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  params_file:="$P" \
  default_nav_to_pose_bt_xml:="$B" \
  map:=/home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces/humble_ws/src/test_gird_map.yaml
```

확인: `bt_navigator` 로그에 `Loaded BT plugin: <타깃>` 출력, lifecycle `active` 도달.
(RViz 는 별도로 `ros2 launch nav2_bringup rviz_launch.py`, 실제 주행 테스트는 Isaac Sim `tb3_env.usd` 실행 필요.)

## 5. 검증되면 원본 반영

플러그인 소스(`custom_nav2_bt_plugins/`)만 원본으로 되돌려 넣는다.
`custom_bt (copy).xml` / `custom_nav2_params (copy).yaml` 의 변경분은 **자동 반영하지 않고**,
검토 후 필요한 부분만 손으로 원본 `custom_bt.xml` / `custom_nav2_params.yaml` 에 옮긴다.

```bash
cd /home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces
rsync -a --exclude build/ --exclude install/ --exclude log/ \
  humble_ws_bttest/src/custom_nav2_bt_plugins/ humble_ws/src/custom_nav2_bt_plugins/

cd humble_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select custom_nav2_bt_plugins
```

## 6. 작업 완료 후: 코드 설명

빌드·테스트가 끝나면, 작성한 코드를 **객체/인스턴스 단위로 하나씩** 설명한다.
전체 요약이 아니라 각 구성요소별로 아래 항목을 채운다.

- **클래스마다**: 이름, 상속한 베이스(`BT::SyncActionNode` / `BtActionNode<T>` 등),
  BT 노드 종류(Action/Condition/Control/Decorator), 존재 이유(무엇을 담당하는가)
- **`providedPorts()` 의 포트마다**: 이름 / 타입 / 방향(input·output) / 기본값 / 의미,
  블랙보드의 어떤 키와 연결되는지
- **멤버 변수(인스턴스)마다**: 타입, 언제 초기화되는지(생성자/`on_tick` 등), 무슨 역할인지
  (예: `node_` = `blackboard->get("node")` 로 받은 rclcpp 노드 핸들, publisher/subscriber/client, 상태 플래그)
- **메서드마다**: `tick()` / `on_tick()` / `on_success()` / `on_aborted()` 등이
  각각 언제 호출되고 어떤 `BT::NodeStatus` 를 반환하며 부수효과가 무엇인지
- **ROS 인터페이스 인스턴스마다**: 토픽/서비스/액션 이름·타입·QoS, 방향, 누가 상대인지
- **등록부**: `BT_REGISTER_NODES` 에서 XML ID ↔ 클래스 매핑, `plugin_lib_names` 의 타깃명

형식은 소스별로 나눠 `src/<name>.cpp`, `include/.../<name>.hpp` 순서로,
클래스 → 포트 → 멤버 → 메서드 순서로 적는다.

## 롤백

- 복사본 빌드 꼬임: `rm -rf humble_ws_bttest/{build,install,log}` 후 재빌드
- 복사본 폐기: `rm -rf humble_ws_bttest` (원본 무영향)
- 원본 소스 되돌리기: `git -C humble_ws/src/custom_nav2_bt_plugins checkout -- .`

## 트러블슈팅

| 증상 | 확인 |
| --- | --- |
| `find_package(behaviortree_cpp)` 실패 | `package.xml`·`CMakeLists.txt` 를 `behaviortree_cpp_v3` 로 |
| `undefined symbol` 로 .so 로드 실패 | `.cpp` 에 `BT_REGISTER_NODES` 매크로 / `ament_target_dependencies` 누락 |
| 헤더 못 찾음 | `include_directories(include)` + `install(DIRECTORY include/ DESTINATION include/)` |
| `bt_navigator` 가 노드 못 찾음 | `custom_nav2_params (copy).yaml` 의 `plugin_lib_names` 에 CMake 타깃명, `install/setup.bash` source |
| 원본 params/xml 이 바뀜 | launch 에 원본 경로를 넘겼는지 확인 — 반드시 `(copy)` 파일만 지정 |
