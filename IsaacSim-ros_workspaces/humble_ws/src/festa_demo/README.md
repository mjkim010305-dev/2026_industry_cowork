# festa_demo — 실기 단독 배포용 AI Festa ㄴ자 코스 sweep 데모

`festa_bringup`(pick:=false 모드)이 런타임에 쓰는 것만 그대로 뽑아, 협업자 패키지
(`custom_nav2_bt_plugins`, `green_box_approach`, `festa_manipulation`,
`turtlebot3_msgs`)도, 이 워크스페이스의 `src/` 레이아웃도 없이 **`/opt/ros/humble`
(+ nav2, cv_bridge, control_msgs, behaviortree_cpp_v3)만으로 빌드·실행**되는
독립 패키지다. 부품을 싣지 않고, 초록 박스가 막으면 접근해 팔로 쓸어(sweep) 치우고
목표까지 갔다가 출발점으로 돌아온다(`festa_bringup`의 pick:=false, return_to_start:=true에서
출발. 다른 점: 컨트롤러를 rotation shim으로 감쌈, 도착 방향 — 아래 "실행" 참고).

## 실기 배포

```bash
# 이 폴더를 통째로 복사한다
cp -r festa_demo ~/turtlebot3_ws/src/

cd ~/turtlebot3_ws
colcon build --packages-select festa_demo
source install/setup.bash
```

## 실행

```bash
# 로봇 bringup(hardware.launch.py + 라이다 + RealSense)이 떠 있을 때
ros2 launch festa_demo festa_demo.launch.py mode:=real \
  initial_x:=<출발 x> initial_y:=<출발 y> initial_yaw:=<rad> \
  goal_x:=<목표 x> goal_y:=<목표 y> goal_yaw:=<rad>

# 시뮬: Isaac에서 코스 씬이 재생 중일 때
ros2 launch festa_demo festa_demo.launch.py mode:=sim
```

- 목표에 도착하면 출발점으로 되돌아온다(`return_to_start:=true`가 기본).
- `goal_yaw`는 도착했을 때 바라볼 방향이다. **진행 방향 그대로 준다.** 돌아올 때의
  180° 회전은 Nav2 rotation shim이 제자리에서 한다. 출발점에는 출발 방향의 반대
  (`initial_yaw + π`)를 바라보며 도착한다.
- DWB만 쓰면 큰 방향 전환을 크게 전진 호를 그리며 해서 모퉁이 장애물·벽에 박았다
  (시뮬 N3·D1·D2). 그래서 컨트롤러를 `nav2_rotation_shim_controller`(DWB 설정 그대로)로
  감쌌다. **실기에서 `ros2 pkg prefix nav2_rotation_shim_controller`로 설치 여부를 먼저 확인.**

**주의:** 협업자의 `festa_manipulation/festa_action/sweep_action_server.py`를 동시에
띄우지 마라. 둘 다 `/sweep`이라는 같은 이름을 쓰지만 액션 타입이 다르다
(`turtlebot3_msgs/action/Sweep` vs 이 패키지의 `festa_demo/action/Sweep`) — 같은
이름에 다른 타입의 서버/클라이언트가 동시에 떠 있으면 접속이 꼬인다.

## 이 패키지가 어디서 왔는지 (원본 경로, humble_ws/src/ 기준)

| festa_demo | 원본 |
|---|---|
| `action/Sweep.action` | `festa_manipulation/turtlebot3_msgs/action/Sweep.action` (그대로) |
| `include/festa_demo/*.hpp`, `src/*.cpp` (4개 BT 플러그인) | `custom_nav2_bt_plugins/{include,src}/.../{final_approach_stop_action,sweep_obstacle_action,is_green_box_detected_condition,compute_green_box_approach_goal_action}.{hpp,cpp}` — namespace `custom_nav2_bt_plugins` → `festa_demo`, 인클루드 경로, SweepObstacle의 액션 타입(`turtlebot3_msgs::action::Sweep` → `festa_demo::action::Sweep`)만 변경. BT 노드 이름(`FinalApproachStop`/`SweepObstacle`/`IsGreenBoxDetected`/`ComputeGreenBoxApproachGoal`)과 포트/동작은 동일 |
| `scripts/detector_node.py` | `green_box_approach/green_box_approach/detector_node.py` — `from green_box_approach.geometry import ...` → `from geometry import ...`만 변경 |
| `scripts/geometry.py` | `green_box_approach/green_box_approach/geometry.py` (그대로) |
| `scripts/obstacle_clear_sequence.py` | `festa_manipulation/festa_action/obstacle_clear_sequence.py` (그대로) |
| `scripts/sweep_only.py` | `festa_bringup/scripts/sweep_only.py`의 `run_sequence()` 부분 — `FESTA_ACTION_DIR`/`server` 모드 없이 같은 폴더에서 `obstacle_clear_sequence`를 import, `P_HOME`은 `festa_manipulation/festa_action/rear_pick.py`에서 복사한 상수 |
| `scripts/sweep_action_server.py` | `festa_manipulation/festa_action/sweep_action_server.py` — `from turtlebot3_msgs.action import Sweep` → `from festa_demo.action import Sweep`, `SEQUENCE_SCRIPT`가 같은 폴더의 `sweep_only.py`를 가리키도록만 변경 |
| `scripts/send_goal.py` | `festa_bringup/festa_bringup/send_goal.py` — 복귀 goal의 방향만 `initial_yaw + π`로 변경 |
| `scripts/moveit_to_isaac_bridge.py` | `moveit_to_isaac_bridge.py` (그대로, 시뮬 전용) |
| `bt/festa_demo.xml` | `festa_bringup/bt/festa_l_course_nopick.xml` (트리는 동일, 헤더 코멘트만 변경) |
| `params/festa_demo_nav2.yaml` | `festa_bringup/params/l_course_nav2.yaml` — `plugin_lib_names`에서 `custom_*` 라이브러리를 모두 빼고 `festa_demo_*` 4개로 교체, `FollowPath`를 rotation shim(DWB 설정 그대로)으로 감쌈 |
| `maps/*` | `festa_bringup/maps/*` (그대로) |
| `launch/festa_demo.launch.py` | `festa_bringup/launch/festa_scenario.launch.py`에서 파생 — `pick` 인자 없음, `ws_src` 인자 없음(bridge/sweep server/detector/send_goal 전부 이 패키지에서 실행) |

## 참고

- 시뮬 검증·실기 미검증 이력, BT 흐름, HSV/박스 크기 튜닝 절차는 `festa_bringup/README.md`를 참고한다(이 패키지는 그 pick:=false 경로의 복사본이다).
