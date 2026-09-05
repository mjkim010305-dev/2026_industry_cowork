# 세션 정리 — custom_nav2_bt_plugins (2026-09-03)

이 세션에서 다룬 주제 요약.

---

## 1. `PLUGIN_DEVELOPMENT_GUIDE.md` 작성

Nav2 BT 커스텀 노드 플러그인을 이 패키지에서 개발하기 위한 문서를 만들었다.
처음엔 전체 개발 가이드로 크게 썼다가, 요청에 따라 **빌드/테스트 런북**으로 축소했다.

- 코드 작성은 이 디렉터리에서 대화하며 진행한다.
- 빌드/테스트가 필요할 때만 `@PLUGIN_DEVELOPMENT_GUIDE.md` 를 멘션해서 절차를 돌린다.
- `CLAUDE.md` 선언은 하지 않기로 함 → 이 md 는 자동으로 읽히지 않고, 멘션할 때만 참조된다.

---

## 2. 환경 조사 결과

| 항목 | 값 |
| --- | --- |
| ROS 배포판 | `humble` |
| colcon | `/usr/bin/colcon` (apt) |
| Nav2 | `/opt/ros/humble` 바이너리, `nav2_behavior_tree` 1.1.20 |
| BT.CPP | **`behaviortree_cpp_v3` 만 설치됨** (v4 `behaviortree_cpp` 없음) |
| 패키지 소스 | `ros2 pkg create` 로 만든 스켈레톤 (초기엔 `include/`, `src/` 비어 있었음) |

### 발견한 불일치

- `package.xml` / `CMakeLists.txt` 가 `behaviortree_cpp` (BT.CPP v4) 에 의존하도록 되어 있었다.
- Humble Nav2 는 `behaviortree_cpp_v3` 를 쓴다.
- 그대로 `colcon build` 하면 `find_package(behaviortree_cpp REQUIRED)` 에서 실패한다.
- → 의존성을 `behaviortree_cpp_v3` 로 바꾸고, v3 API(`behaviortree_cpp_v3/...` 헤더, `BT_REGISTER_NODES` 매크로)로 작성하기로 결정.
- `custom_bt.xml` 도 v3 문법(`<root main_tree_to_execute="MainTree">`)을 쓰고 있어 v3 방향이 맞다.

---

## 3. 원본 보존 · 사본에서 테스트 (작업 방침)

내일 이어서 작업하기 때문에 원본을 바꾸면 안 된다. 그래서:

- colcon 빌드·테스트는 **복사본 워크스페이스 `humble_ws_bttest`** 에서만 한다.
  원본 `humble_ws/{build,install,log}` 는 건드리지 않는다.
- BT 트리·파라미터 테스트는 원본이 아니라 **사본 파일**에서만 편집·실행한다:
  - `custom_bt (copy).xml`
  - `custom_nav2_params (copy).yaml`
- `ros2 launch nav2_bringup bringup_launch.py` 에 `params_file:=` 와
  `default_nav_to_pose_bt_xml:=` 로 이 사본 경로를 넘긴다 (공백 파일명이라 따옴표 필수).
- 검증 후 원본 반영 시: 플러그인 소스 디렉터리만 rsync 하고,
  params/xml 변경분은 검토 후 손으로 원본에 옮긴다.

가이드에는 사본 생성 절(1-1), rsync `--exclude` 로 사본 보호, launch 예시(4-1),
"원본 params/xml 이 바뀜" 트러블슈팅 행이 추가되어 있다.

---

## 4. git log 관련 확인

작업 중 `git log --oneline` 에 커밋이 많이 보여서 확인했다.

- 저장소 루트는 `.../IsaacSim-ros_workspaces` 하나이고, remote 는
  `https://github.com/isaac-sim/IsaacSim-ros_workspaces.git` (NVIDIA 공식).
- 전체 커밋 39개는 전부 **clone 시점부터 있던 업스트림 히스토리**
  (`Initial commit` → `Updates for Isaac Sim 2023.1.0 … 6.0.1`).
- `Squashed commit of the following:` 메시지들은 isaac-sim 팀의 릴리스 관례이지 이 세션에서 만든 게 아니다.
- `reflog` 전체가 `clone` 한 줄뿐 → 이 세션에서 커밋/rebase/reset 을 한 적 없다.
- `git log --oneline -- .` (이 폴더 한정) → 아무것도 안 나옴.
  `custom_nav2_bt_plugins/` 는 아직 한 번도 커밋되지 않은 **untracked 폴더**.
- `ros2 pkg create` 는 git 커밋을 만들지 않는다. 디스크에 파일만 생성한다.

---

## 5. Nav2 launch 로 테스트 가능 여부

| 명령 | 가능성 | 비고 |
| --- | --- | --- |
| `bringup_launch.py` | 백그라운드로 띄우고 로그·lifecycle 확인 후 종료 가능 | `use_sim_time:=true` 는 `/clock` 퍼블리셔(Isaac Sim)가 없으면 노드가 clock 대기하며 멈춤 → 로딩 확인만 하려면 `use_sim_time:=false` |
| `rviz_launch.py` | 띄울 순 있으나 화면을 볼 수 없음 | GUI 확인은 사용자가 직접 |
| goal 보내서 실제 주행 | 불가 | TF/odom/scan/clock 소스로 Isaac Sim `tb3_env.usd` 가 돌아야 함 |

- 맵 파일: `src/test_gird_map.yaml` (파일명 오타 "gird"), 이미지 `test_grid_map.png`, resolution 0.05.
- BT 플러그인 테스트의 핵심 확인 지점: `bt_navigator` 로그의 `Loaded BT plugin: <타깃>` 과 lifecycle `active` 도달.

---

## 6. 가이드에 추가한 "작업 완료 후 코드 설명" 규칙

빌드·테스트가 끝나면 작성한 코드를 **객체/인스턴스 단위로 하나씩** 설명한다 (전체 요약 금지).

- **클래스마다**: 이름, 상속한 베이스, BT 노드 종류(Action/Condition/Control/Decorator), 담당 역할
- **`providedPorts()` 포트마다**: 이름 / 타입 / 방향 / 기본값 / 의미, 연결되는 블랙보드 키
- **멤버 변수마다**: 타입, 초기화 시점(생성자 / `onStart` 등), 역할
- **메서드마다**: `tick()` / `onStart()` / `onRunning()` / `onHalted()` 등의 호출 시점, 반환 `BT::NodeStatus`, 부수효과
- **ROS 인터페이스 인스턴스마다**: 토픽/서비스/액션 이름·타입·QoS·방향·상대
- **등록부**: `BT_REGISTER_NODES` 의 XML ID ↔ 클래스 매핑, `plugin_lib_names` 타깃명

서술 순서: 소스 파일별(`.cpp` → `.hpp`), 각 파일 안에서 클래스 → 포트 → 멤버 → 메서드.

---

## 7. 개념 Q&A — 토픽 이름 vs 메시지 타입/규격

**질문:** 토픽 이름은 XML(→cpp `getInput`)에서 바꾸는 걸 알겠는데, 왜 메시지 타입과 규격(`{}`)까지 다 맞춰서 줘야 하나?

**요점:** 토픽 이름은 "주소"일 뿐이고, 실제 인터페이스(계약)는 메시지 타입이다.

- 토픽으로 흐르는 건 바이트열이다. 이름은 라우팅 라벨일 뿐, 해석 방법을 알려주지 않는다.
- ROS 2 는 정적 타입 + 코드 생성 방식이다. `.msg` 가 `rosidl` 로 구조체 + CDR 직렬화기로 컴파일된다.
  직렬화는 **레이아웃(필드 순서) 기반**이고 자기설명 정보가 기본적으로 없다.
  수신측은 자기가 아는 타입 코드로 위치 순서대로 역직렬화하므로 송신측이 정확히 그 레이아웃을 만들어야 한다.
- DDS 는 **토픽 이름 + 타입명 + 타입 해시(RIHS) + QoS** 가 모두 호환돼야 연결한다.
  타입이 다르면 깨진 데이터가 아니라 **연결 자체가 안 되고 콜백이 조용히 안 불린다.**
- 메시지는 필드 집합이 고정된 구조체다. `publish()` 에는 "완성된 구조체 하나"를 넘긴다.
  `std_msgs::msg::Bool{}` 는 `data=false` 로 채워진 완전한 객체다.
  관심 없는 필드는 안 건드려도 되지만(기본값), **와이어에는 항상 전체 구조체가 실린다.**
  수신측이 어떤 필드(예: `Header.frame_id`)를 필요로 하면 채우지 않으면 알 방법이 없다.

| 요소 | 누가 정하나 | XML 로 변경 |
| --- | --- | --- |
| 토픽 이름 | cpp `getInput("topic", ...)` → XML 포트 | O |
| 메시지 타입 | cpp `create_subscription<T>` 의 `T` | X (컴파일 타임 고정) |
| 메시지 필드값 | 퍼블리셔 코드 | 매 publish 마다 전체 구조체 제공 |

예: `wait_for_obstacle_clearance_action.cpp` 는 `create_subscription<std_msgs::msg::Bool>(topic_, ...)`.
`topic_` 은 XML 포트에서 오지만, 타입 `std_msgs/msg/Bool` 은 고정이다.
이 노드에 obstacle 상태를 물려주려면 반드시 `std_msgs/msg/Bool` 을 그 토픽에 publish 해야 한다.

---

## 현재 상태 / 다음 할 일

- `src/` 에 실제 플러그인 코드가 생기기 시작함 (`wait_for_obstacle_clearance_action.cpp`,
  `rgbd_obstacle_localizer_node.cpp` 등 — 이 세션 대화 중 사용자가 편집 중).
- 아직 `package.xml` / `CMakeLists.txt` 의 `behaviortree_cpp` → `behaviortree_cpp_v3` 수정은 확인 필요.
- 빌드/테스트는 `humble_ws_bttest` 사본 워크스페이스 + `(copy)` 설정 파일로 진행.
