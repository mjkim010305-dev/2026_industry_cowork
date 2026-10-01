# P_REAR_CARRY → P_REAR_PLACE → G_OPEN
# P_REAR_PRE_SWEEP → P_SIDE_PRE_SWEEP → P_PRE_SWEEP → P_CONTACT
# P_SWEEP_END 실행 -> 임시 안전 컷오프 실행
# P_PRE_SWEEP → P_SIDE_PRE_SWEEP → P_REAR_PRE_SWEEP → P_REAR_PICK → G_GRASP → P_REAR_CARRY

#!/usr/bin/env python3

import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


# ROS 2 인터페이스
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

JOINT_STATE_TOPIC = "/joint_states"
DYNAMIC_JOINT_STATE_TOPIC = "/dynamic_joint_states"
CONTROLLER_STATE_TOPIC = "/arm_controller/controller_state"

SWEEP_STAGE_TOPIC = "/sweep_stage"

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]


# 동작 설정
ARM_MOVE_TIME = 2.0
WAIT_AFTER_GRIP = 1.0
# festa_demo: 4.0 -> 6.0 (60도까지 쓸 때 joint1 추종 오차가 컷오프 0.045를 살짝 넘음, 사용자: 6초로)
SWEEP_MOVE_TIME = 6.0

GRIPPER_MAX_EFFORT = 0.0

G_OPEN = 0.02501155674647538
G_GRASP = 0.002

START_TOLERANCE = 0.12


# 기존 실험의 임시 안전 컷오프
TEMP_CURRENT_CUTOFF_RAW = 90.0
TEMP_ERROR_CUTOFF_RAD = 0.045
TEMP_PERSISTENCE_SEC = 0.15

# 센서 데이터 수신 상태 확인
SENSOR_MAX_AGE_SEC = 0.5


# 후방 운반 자세
P_REAR_CARRY = [
    -3.0234761329225988,
    -0.9802137234589247,
     0.5016117176386047,
     0.2193592526676467,
]


# 후방 Pick 및 Place 공통 자세
P_REAR_PICK = [
    -3.0234761329225988,
     0.679553489033339,
    -0.19634954084936207,
     0.2193592526676467,
]

P_REAR_PLACE = P_REAR_PICK


# 후방 Sweep 전환 준비 자세
P_REAR_PRE_SWEEP = [
    -3.0234761329225988,
    -0.11044661672776616,
    -0.0030679615757712823,
     0.2193592526676467,
]


# 측면 Sweep 전환 자세
P_SIDE_PRE_SWEEP = [
    -1.4940972874006144,
     0.6519418348513975,
    -0.25157284921324513,
    -0.260776733940559,
]


# 기존 FESTA Sweep 준비 자세
P_PRE_SWEEP = [
    -0.8452234141,
     0.4770680250,
    -0.5905826033,
     0.1549320596,
]


# 기존 FESTA Sweep 접촉 자세
# festa_demo (2026-10-01, 사용자: 17 cm에서 시작해 15 cm로 끝나게, 팔을 더 빼서): 손끝 높이
# 21.8 -> 17.0 cm, 도달거리 +2 cm(이 높이에서 joint3 한계 안 최대). joint1·그리퍼 각도는
# 원래 그대로, joint2~4만 로봇 URDF 역기구학으로 조정.
# 원래 값 [-0.4632621979, 0.7669903939, -0.4601942364, -0.3282718886]
P_CONTACT = [
    -0.4632621979,
     1.1569699208,
    -0.8310496882,
    -0.3473959637,
]


# 기존 FESTA Sweep 종료 자세
# festa_demo: 손끝 높이 15.9 -> 15.0 cm, 도달거리 +2 cm (위와 같은 방법). joint1 0.7056 -> 1.05
# (끝 각도 40 -> 60도, 사용자: 비스듬한 박스가 덜 밀림 -> 더 끝까지 쓸기).
# 원래 값 [0.7056311624, 0.7501166053, -0.1580000212, -0.3819612162]
P_SWEEP_END = [
     1.05,
     0.9419997321,
    -0.4635389616,
    -0.2683054026,
]


# 기존 FESTA Sweep 후퇴 자세
P_RETREAT = [
     0.7025632009,
     0.2270291566,
    -0.1027767128,
    -0.3681553891,
]


# festa_demo (2026-10-01, 사용자: 뒤에서 앞으로·앞에서 뒤로 갈 때 팔이 바깥으로 빠져 벽에 닿음,
# 인코스로): 팔을 P_HOME 모양(접힌 자세, rear_pick.py의 P_HOME joint2~4)으로 접은 채
# joint1만 돌리고, 도착한 쪽에서 편다.
ARM_FOLD = [-1.0461748973380072, 1.0753205323078345, 0.009203884727313847]
P_REAR_FOLD = [P_REAR_CARRY[0]] + ARM_FOLD
P_PRE_SWEEP_FOLD = [P_PRE_SWEEP[0]] + ARM_FOLD
P_RETREAT_FOLD = [P_RETREAT[0]] + ARM_FOLD


class ObstacleClearSequence(Node):

    def __init__(self, safety_enabled=True):

        super().__init__("obstacle_clear_sequence")

        # False: Sweep 중 전류/추종 오차 감시를 끈다 (--no-safety).
        # Isaac Sim처럼 /dynamic_joint_states 전류 값이 없는 환경용
        self.safety_enabled = safety_enabled

        if not self.safety_enabled:
            self.get_logger().warning(
                "임시 안전 컷오프 비활성화 (--no-safety): "
                "시뮬레이션에서만 사용하세요."
            )

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION,
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION,
        )

        self.stage_publisher = self.create_publisher(
            String,
            SWEEP_STAGE_TOPIC,
            10,
        )

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            DynamicJointState,
            DYNAMIC_JOINT_STATE_TOPIC,
            self.dynamic_joint_state_callback,
            20,
        )

        self.create_subscription(
            JointTrajectoryControllerState,
            CONTROLLER_STATE_TOPIC,
            self.controller_state_callback,
            20,
        )

        self.current_pose = None

        self.latest_joint1_current_abs = None
        self.latest_joint1_error_abs = None

        self.latest_current_time = None
        self.latest_error_time = None

        self.latest_arm_actual = None

        self.monitor_safety = False

        self.violation_start_time = None

        self.safety_triggered = False
        self.safety_reason = ""

    # 현재 동작 단계 발행
    def publish_stage(self, stage):

        msg = String()
        msg.data = stage

        self.stage_publisher.publish(msg)

        self.get_logger().info(
            f"STAGE -> {stage}"
        )

    # 현재 관절 위치 수신
    def joint_state_callback(self, msg):

        if not all(name in msg.name for name in JOINT_NAMES):
            return

        self.current_pose = [
            float(msg.position[msg.name.index(name)])
            for name in JOINT_NAMES
        ]

    # joint1 전류 수신
    def dynamic_joint_state_callback(self, msg):

        if "joint1" not in msg.joint_names:
            return

        joint_index = msg.joint_names.index("joint1")

        if joint_index >= len(msg.interface_values):
            return

        interface = msg.interface_values[joint_index]

        current_value = None

        preferred_names = [
            "current",
            "present_current",
            "current_raw",
        ]

        for name in preferred_names:

            if name in interface.interface_names:

                index = interface.interface_names.index(name)

                if index < len(interface.values):
                    current_value = float(interface.values[index])

                break

        if current_value is None:

            for index, name in enumerate(interface.interface_names):

                if "current" in name.lower():

                    if index < len(interface.values):
                        current_value = float(interface.values[index])

                    break

        if current_value is None:
            return

        if not math.isfinite(current_value):
            return

        self.latest_joint1_current_abs = abs(current_value)
        self.latest_current_time = time.monotonic()

    # joint1 관절 추종 오차 및 실제 관절 위치 수신
    def controller_state_callback(self, msg):

        if "joint1" not in msg.joint_names:
            return

        name_to_index = {
            name: index
            for index, name in enumerate(msg.joint_names)
        }

        if all(name in name_to_index for name in JOINT_NAMES):

            if len(msg.feedback.positions) >= len(msg.joint_names):

                positions = [
                    float(msg.feedback.positions[name_to_index[name]])
                    for name in JOINT_NAMES
                ]

                if all(math.isfinite(value) for value in positions):
                    self.latest_arm_actual = positions

        index = name_to_index["joint1"]

        if (
            index >= len(msg.reference.positions)
            or index >= len(msg.feedback.positions)
        ):
            return

        command = float(msg.reference.positions[index])
        actual = float(msg.feedback.positions[index])

        if not math.isfinite(command) or not math.isfinite(actual):
            return

        self.latest_joint1_error_abs = abs(command - actual)
        self.latest_error_time = time.monotonic()

    # Action Server 연결 확인
    def wait_servers(self):

        if not self.arm_client.wait_for_server(timeout_sec=10.0):

            self.get_logger().error(
                "Arm Action Server 없음"
            )
            return False

        if not self.gripper_client.wait_for_server(timeout_sec=10.0):

            self.get_logger().error(
                "Gripper Action Server 없음"
            )
            return False

        return True

    # 센서 데이터가 최근에 수신되었는지 확인
    def sensor_data_ready(self):

        now = time.monotonic()

        if self.latest_joint1_current_abs is None:
            return False

        if self.latest_joint1_error_abs is None:
            return False

        if self.latest_current_time is None:
            return False

        if self.latest_error_time is None:
            return False

        if now - self.latest_current_time > SENSOR_MAX_AGE_SEC:
            return False

        if now - self.latest_error_time > SENSOR_MAX_AGE_SEC:
            return False

        return True

    # Sweep 시작 전 센서 연결 확인
    def wait_for_sensor_data(self):

        end_time = time.monotonic() + 3.0

        while rclpy.ok() and time.monotonic() < end_time:

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            if self.sensor_data_ready():
                return True

        self.get_logger().error(
            "전류 또는 관절 추종 오차 데이터 수신 실패"
        )

        return False

    # 현재 자세 확인
    def check_pose(self, expected, label):

        self.current_pose = None

        for _ in range(30):

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            if self.current_pose is not None:
                break

        if self.current_pose is None:

            self.get_logger().error(
                "/joint_states 수신 실패"
            )
            return False

        errors = [
            abs(current - target)
            for current, target in zip(
                self.current_pose,
                expected,
            )
        ]

        if max(errors) > START_TOLERANCE:

            self.get_logger().error(
                f"현재 자세가 {label}과 다릅니다."
            )

            self.get_logger().error(
                f"현재 관절값: {self.current_pose}"
            )

            return False

        self.get_logger().info(
            f"{label} 확인 완료"
        )

        return True

    # 이동 시간 메시지 생성
    @staticmethod
    def make_duration(seconds):

        sec = int(seconds)

        return Duration(
            sec=sec,
            nanosec=int(
                (seconds - sec) * 1_000_000_000
            ),
        )

    # 임시 안전 컷오프 감시
    def check_temporary_safety_cutoff(self):

        if not self.monitor_safety:
            return

        if self.safety_triggered:
            return

        if not self.sensor_data_ready():

            self.safety_triggered = True
            self.safety_reason = "전류 또는 관절 상태 데이터 수신 중단"

            self.get_logger().error(
                "센서 데이터 수신 중단으로 Sweep 정지 요청"
            )

            return

        current_over = (
            self.latest_joint1_current_abs
            >= TEMP_CURRENT_CUTOFF_RAW
        )

        error_over = (
            self.latest_joint1_error_abs
            >= TEMP_ERROR_CUTOFF_RAD
        )

        now = time.monotonic()

        if not current_over and not error_over:

            self.violation_start_time = None
            return

        if self.violation_start_time is None:

            self.violation_start_time = now
            return

        duration = now - self.violation_start_time

        if duration < TEMP_PERSISTENCE_SEC:
            return

        self.safety_triggered = True

        reasons = []

        if current_over:

            reasons.append(
                f"joint1 current_abs="
                f"{self.latest_joint1_current_abs:.2f}"
            )

        if error_over:

            reasons.append(
                f"joint1 error_abs="
                f"{self.latest_joint1_error_abs:.6f}"
            )

        self.safety_reason = ", ".join(reasons)

        self.get_logger().error(
            "임시 안전 컷오프 발생: "
            f"{self.safety_reason}, "
            f"지속시간={duration:.3f}s"
        )

    # 실행 중인 Sweep Goal 취소 및 현재 자세 유지
    def cancel_and_hold(self, goal_handle, result_future):

        self.get_logger().warning(
            "Sweep Goal 취소 요청"
        )

        self.monitor_safety = False

        cancel_future = goal_handle.cancel_goal_async()

        rclpy.spin_until_future_complete(
            self,
            cancel_future,
            timeout_sec=2.0,
        )

        if not cancel_future.done():

            self.get_logger().error(
                "Goal 취소 응답을 받지 못했습니다."
            )

            return False

        cancel_result = cancel_future.result()

        if cancel_result is None:

            self.get_logger().error(
                "Goal 취소 결과가 없습니다."
            )

            return False

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=3.0,
        )

        if not result_future.done():

            self.get_logger().error(
                "기존 Sweep Goal 종료를 확인하지 못했습니다."
            )

            return False

        final_result = result_future.result()

        if final_result is None:

            self.get_logger().error(
                "Sweep 종료 결과가 없습니다."
            )

            return False

        self.get_logger().warning(
            f"Sweep 종료 상태: {final_result.status}"
        )

        if final_result.status not in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_ABORTED,
        ):

            self.get_logger().error(
                "Sweep Goal이 종료되었는지 확인할 수 없습니다."
            )

            return False

        if self.latest_arm_actual is None:

            self.get_logger().error(
                "실제 관절 위치가 없어 HOLD를 전송할 수 없습니다."
            )

            return False

        self.get_logger().warning(
            "현재 관절 위치로 Software HOLD 전송"
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()

        point.positions = self.latest_arm_actual.copy()
        point.time_from_start = self.make_duration(0.2)

        goal.trajectory.points = [point]

        hold_future = self.arm_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            hold_future,
            timeout_sec=2.0,
        )

        if not hold_future.done():
            return False

        hold_handle = hold_future.result()

        if hold_handle is None or not hold_handle.accepted:
            return False

        hold_result_future = hold_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            hold_result_future,
            timeout_sec=3.0,
        )

        if not hold_result_future.done():
            return False

        hold_result = hold_result_future.result()

        if hold_result is None:
            return False

        if (
            hold_result.status != GoalStatus.STATUS_SUCCEEDED
            or hold_result.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            return False

        self.get_logger().warning(
            "Software HOLD 명령 완료"
        )

        return True

    # 로봇팔 이동
    def move_arm(self, pose, label, monitor=False, duration=None):

        self.get_logger().info(
            f"팔 이동 시작: {label}"
        )

        # 안전 감시 비활성화 시 센서 대기와 컷오프 판정을 모두 건너뜀
        if not self.safety_enabled:
            monitor = False

        self.monitor_safety = False
        self.violation_start_time = None
        self.safety_triggered = False
        self.safety_reason = ""

        if monitor:

            if not self.wait_for_sensor_data():
                return "failed"

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()

        point.positions = pose

        move_time = ARM_MOVE_TIME if duration is None else duration

        point.time_from_start = self.make_duration(
            move_time
        )

        goal.trajectory.points = [point]

        send_future = self.arm_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():

            self.get_logger().error(
                f"{label}: Goal 전송 시간 초과"
            )

            return "failed"

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:

            self.get_logger().error(
                f"{label}: Goal 거절"
            )

            return "failed"

        result_future = goal_handle.get_result_async()

        self.monitor_safety = monitor

        # Sweep 실행 중에만 전류 및 추종 오차 감시
        while rclpy.ok() and not result_future.done():

            rclpy.spin_once(
                self,
                timeout_sec=0.01,
            )

            if monitor:

                self.check_temporary_safety_cutoff()

                if self.safety_triggered:

                    hold_ok = self.cancel_and_hold(
                        goal_handle,
                        result_future,
                    )

                    if not hold_ok:

                        self.get_logger().error(
                            "정지 명령 완료를 확인하지 못했습니다."
                        )

                        self.get_logger().error(
                            "로봇 상태를 확인하고 수동으로 조치하세요."
                        )

                    return "safety_stop"

        self.monitor_safety = False

        if not result_future.done():

            self.get_logger().error(
                f"{label}: Goal 종료 확인 실패"
            )

            return "failed"

        result = result_future.result()

        if result is None:

            self.get_logger().error(
                f"{label}: 실행 결과 수신 실패"
            )

            return "failed"

        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):

            self.get_logger().error(
                f"{label}: 이동 실패, "
                f"status={result.status}, "
                f"error_code={result.result.error_code}"
            )

            return "failed"

        self.get_logger().info(
            f"{label}: 이동 완료"
        )

        return "ok"

    # 그리퍼 열기 또는 닫기
    def move_gripper(self, position, label):

        self.get_logger().info(
            f"그리퍼 동작 시작: {label}"
        )

        goal = GripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        send_future = self.gripper_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():
            return False

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=15.0,
        )

        if not result_future.done():
            return False

        result = result_future.result()

        if result is None:
            return False

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            return False

        grip_result = result.result

        self.get_logger().info(
            f"{label} 완료: "
            f"position={grip_result.position}, "
            f"reached_goal={grip_result.reached_goal}, "
            f"stalled={grip_result.stalled}"
        )

        if label == "G_OPEN" and not grip_result.reached_goal:
            return False

        return True

    # 후방 Place
    def run_rear_place(self):

        self.publish_stage("REAR_PLACE")

        if self.move_arm(
            P_REAR_PLACE,
            "P_REAR_PLACE",
        ) != "ok":
            return False

        if not self.move_gripper(
            G_OPEN,
            "G_OPEN",
        ):
            return False

        time.sleep(WAIT_AFTER_GRIP)

        return True

    # 후방에서 전방 Sweep 준비 자세로 이동
    def run_prepare_sweep(self):

        steps = [
            ("P_REAR_PRE_SWEEP", P_REAR_PRE_SWEEP),
            ("P_REAR_FOLD", P_REAR_FOLD),            # festa_demo: 접고
            ("P_PRE_SWEEP_FOLD", P_PRE_SWEEP_FOLD),  # festa_demo: 접은 채 앞으로 회전
            ("P_PRE_SWEEP", P_PRE_SWEEP),
        ]

        for label, pose in steps:

            self.publish_stage(label)

            if self.move_arm(pose, label) != "ok":
                return False

        return True

    # 기존 전방 Sweep 및 임시 안전 컷오프
    def run_sweep(self):

        self.publish_stage("APPROACH_CONTACT")

        if self.move_arm(
            P_CONTACT,
            "P_CONTACT",
        ) != "ok":

            self.publish_stage("FAILED")
            return False

        self.publish_stage("SWEEP_AFTER_CONTACT")

        result = self.move_arm(
            P_SWEEP_END,
            "P_SWEEP_END",
            monitor=True,
            duration=SWEEP_MOVE_TIME,
        )

        if result == "safety_stop":

            self.publish_stage("SAFETY_STOP")

            self.get_logger().error(
                "임시 안전 컷오프 또는 센서 이상으로 Sweep 중단"
            )

            self.get_logger().error(
                f"중단 원인: {self.safety_reason}"
            )

            self.get_logger().error(
                "자동 후퇴 및 후방 재파지는 실행하지 않습니다."
            )

            return False

        if result != "ok":

            self.publish_stage("FAILED")
            return False

        self.publish_stage("RETREAT")

        if self.move_arm(
            P_RETREAT,
            "P_RETREAT",
        ) != "ok":

            self.publish_stage("FAILED")
            return False

        self.publish_stage("SWEEP_COMPLETE")

        return True

    # 전방 Sweep 이후 후방 Pick 자세로 이동
    def run_prepare_repick(self):

        steps = [
            ("P_RETREAT_FOLD", P_RETREAT_FOLD),      # festa_demo: 접고 (원래 P_PRE_SWEEP)
            ("P_REAR_FOLD", P_REAR_FOLD),            # festa_demo: 접은 채 뒤로 회전
            ("P_REAR_PRE_SWEEP", P_REAR_PRE_SWEEP),
            ("P_REAR_PICK", P_REAR_PICK),
        ]

        for label, pose in steps:

            self.publish_stage(label)

            if self.move_arm(pose, label) != "ok":
                return False

        return True

    # 후방 물체 재파지
    def run_rear_repick(self):

        self.publish_stage("REAR_REPICK")

        if not self.move_gripper(
            G_GRASP,
            "G_GRASP",
        ):
            return False

        time.sleep(WAIT_AFTER_GRIP)

        if self.move_arm(
            P_REAR_CARRY,
            "P_REAR_CARRY",
        ) != "ok":
            return False

        self.publish_stage("REAR_REPICK_COMPLETE")

        return True

    # 후방 Place부터 Sweep까지 실행
    def run_clear(self):

        if not self.check_pose(
            P_REAR_CARRY,
            "P_REAR_CARRY",
        ):
            return False

        if not self.run_rear_place():
            return False

        if not self.run_prepare_sweep():
            return False

        if not self.run_sweep():
            return False

        return True

    # 정상 Sweep 완료 자세에서 후방 재파지까지 실행
    def run_recover(self):

        if not self.check_pose(
            P_RETREAT,
            "P_RETREAT",
        ):
            return False

        if not self.run_prepare_repick():
            return False

        if not self.run_rear_repick():
            return False

        return True

    # 전체 동작 자동 실행
    def run_all(self):

        self.publish_stage("SEQUENCE_START")

        if not self.run_clear():

            self.publish_stage("SEQUENCE_FAILED")
            return False

        if not self.run_recover():

            self.publish_stage("SEQUENCE_FAILED")
            return False

        self.publish_stage("SEQUENCE_COMPLETE")

        return True


def main():

    args = sys.argv[1:]

    safety_enabled = "--no-safety" not in args

    args = [arg for arg in args if arg != "--no-safety"]

    if (
        len(args) != 1
        or args[0] not in ["all", "clear", "recover"]
    ):

        print("사용법:")
        print("python3 obstacle_clear_sequence.py clear")
        print("python3 obstacle_clear_sequence.py recover")
        print("python3 obstacle_clear_sequence.py all")
        print("시뮬레이션(전류 토픽 없음): 뒤에 --no-safety 추가")

        return

    mode = args[0]

    rclpy.init()

    node = ObstacleClearSequence(safety_enabled=safety_enabled)

    success = False

    try:

        if node.wait_servers():

            if mode == "clear":
                success = node.run_clear()

            elif mode == "recover":
                success = node.run_recover()

            elif mode == "all":
                success = node.run_all()

        if not success:

            node.publish_stage("SEQUENCE_FAILED")

            node.get_logger().error(
                "동작 실패 또는 중단"
            )

    except KeyboardInterrupt:

        node.publish_stage("INTERRUPTED")

        node.get_logger().warning(
            "사용자가 프로그램을 중단했습니다."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()