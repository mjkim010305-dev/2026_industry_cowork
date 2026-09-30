# festa_bringup — AI Festa ㄴ자 코스 원큐 런치

부품을 로봇 뒤에 싣고 코스 끝으로 간다. 가는 길을 **초록 박스가 막고 있으면**
박스 앞에 멈추고 다음 순서로 처리한 뒤 목표까지 간다.

1. 부품을 뒤에 내려놓는다.
2. 팔을 좌우로 쓸어(sweep) 박스를 치운다.
3. 로봇이 움직이지 않은 채 부품을 다시 집는다.

박스 옆으로 비켜 갈 공간이 있으면 치우지 않고 그냥 지나간다.

```bash
# 시뮬: Isaac에서 코스 씬이 재생 중일 때
ros2 launch festa_bringup festa_scenario.launch.py mode:=sim

# 실기: 로봇 bringup(hardware.launch.py + 라이다 + RealSense)이 떠 있을 때
ros2 launch festa_bringup festa_scenario.launch.py mode:=real \
  initial_x:=<출발 x> initial_y:=<출발 y> initial_yaw:=<rad> \
  goal_x:=<목표 x> goal_y:=<목표 y> goal_yaw:=<rad>
```

## 이 런치가 띄우는 것

| 순서 | 무엇 | 비고 |
|---|---|---|
| 1 | `moveit_to_isaac_bridge.py` | sim만 해당. 실기는 ros2_control의 `/arm_controller`가 대신한다 |
| 2 | base_link → camera_optical static TF | sim만 해당(`publish_camera_tf`) |
| 3 | Nav2 bringup | 맵 `maps/l_course_{sim,real}.yaml`, 파라미터 `params/l_course_nav2.yaml`, BT `bt/festa_l_course.xml` |
| 4 | `green_box_approach` 검출기 | HSV + 라이다 → `green_box/pose`, `green_box/detected` |
| 5 | `festa_action/sweep_action_server.py` | `/sweep` 서버. 안전 컷오프는 실기 on, 시뮬 off(Isaac에는 전류 값이 없음) |
| 6 | `send_goal` | `goal_delay`초(기본 20) 뒤 초기 자세를 주고 목표를 한 번 보낸다. 결과는 로그에 `Goal finished with status: ...`로 남는다 |

- 로봇 bringup(Isaac, 또는 `hardware.launch.py` + 라이다 + 카메라)은 이 런치에 포함되지 않는다.
- `sweep_action_server.py`와 `moveit_to_isaac_bridge.py`는 설치되는 파일이 아니라 워크스페이스 `src/` 아래 스크립트다. 런치는 이 패키지 install 경로 옆의 `src/`에서 찾으며, 다른 곳이면 `ws_src:=`로 지정한다.

## BT 흐름 (`bt/festa_l_course.xml`)

협업자의 `L_corridor_sweep.xml`을 바탕으로 **"막혔을 때만 치움"**을 더했다.

- 목표까지 1 Hz로 경로를 계산한다. global costmap에 라이다 장애물 층이 있어서 박스가 보이면 costmap에 들어간다.
  - 계획이 **성공**하면 `goal_blocked=false`: 박스를 비켜 가는 경로여도 그대로 따라간다.
  - 계획이 **실패**하면 `goal_blocked=true`.
- **초록 박스 검출 AND 막힘**일 때만 다음을 실행한다. 목표 도착까지 1회만 한다.
  1. 박스 0.8 m 앞까지 이동
  2. `FinalApproachStop`: 박스 중심까지 0.24 m가 될 때까지 0.08 m/s로 접근
  3. 2초 정지
  4. `/sweep` 호출
  5. costmap 초기화
- 막혔는데 초록 박스가 아직 확정되지 않았으면 제자리에서 기다린다. 옛 경로로 밀고 들어가거나 recovery Spin을 돌지 않는다.

## 시뮬 검증 (2026-09-30~10-01, Isaac, 협업자 `L_corridor_green_box.usd` + 박스 크기 변경)

실행 로그: `mona-scenes/logs/lcourse_L*`

- **박스 0.185 m 정육면체:** 원큐 런치로 부품 집기 → 검출 → 막힘 판정 → 접근·정지 → sweep(박스를 왼쪽으로 0.33~0.39 m 이동) → 목표 도착(SUCCEEDED)까지 확인했다.
- **박스 높이 0.12 m(사용자 추정치):** 라이다 스캔 평면(바닥 위 약 0.12~0.14 m)보다 낮아 costmap에 들어가지 않는다. 그래서 막힘 판정이 나지 않았고, 로봇이 차체로 박스를 밀고 지나갔다(L1).
  - sweep 자세의 손가락 높이도 0.16~0.24 m라 12 cm 박스에는 닿지 않는다.
  - **실제 박스 높이 확인이 필요하다.**
- 시뮬의 부품 집기는 **자석식 연출**이다. 시뮬 그리퍼가 부품을 잡지 못해서, 손가락이 닫히면 부품을 붙이는 방식으로 대신했다.

## 실기 전 확인할 것 (아직 실기에서 돌려 보지 않음)

1. 카메라 토픽과 프레임이 기본값과 맞는지 확인한다: `/camera/camera/color/image_raw`, `camera_color_optical_frame`. TF `base_link → camera_color_optical_frame`이 이어져 있어야 한다.
2. `maps/l_course_real.yaml`(협업자 `real.pgm`)에서 출발 자세와 목표 좌표를 정해 인자로 넘긴다. 기본값 0,0은 자리표시자다.
3. `box_depth`/`box_height`를 실측값으로 준다. 기본값 0.185 / 0.12는 사용자 추정치다.
4. `stop_center_distance`(0.24 m)는 시뮬 기준값이다. 실기 검출 오차에 따라 조정이 필요할 수 있다.
