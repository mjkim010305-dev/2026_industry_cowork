#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


# ============================================================
# 확인된 ROS 인터페이스
# ============================================================

ARM_ACTION_NAME = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION_NAME = "/gripper_controller/gripper_cmd"

JOINT_STATE_TOPIC = "/joint_states"

# Recorder가 실제 명령값을 받을 토픽
ACT_ACTION_TOPIC = "/act/action"

ARM_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]

ACTION_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "gripper_left_joint",
]


# ============================================================
# URDF에서 확인된 팔 관절 제한
# 끝단 충돌을 피하기 위해 소프트웨어 여유값을 둔다.
# ============================================================

ARM_LIMITS: List[Tuple[float, float]] = [
    (-3.07432619, 3.07432619),   # joint1
    (-1.790707812546182, 1.5707963267948966), # joint2
    (-0.9424777960769379, 1.382300767579509), # joint3
    (-1.790707812546182, 2.0420352248333655), # joint4
]

JOINT_LIMIT_MARGIN = 0.02


# ============================================================
# 기존 성공 스크립트에서 확인된 그리퍼 값
# ============================================================

G_OPEN = 0.02501155674647538
G_GRASP = 0.002
GRIPPER_MAX_EFFORT = 0.0


# ============================================================
# 첫 무부하 시험용 조정값
# 하드웨어 고정값이 아니라 실험 후 조정할 값
# ============================================================

# 키 1회당 팔 관절 변화량: 약 1.72도
JOINT_STEP = 0.03

# 한 번의 관절 이동 완료 시간
PATH_TIME = 0.8

# 지수이동평균 계수
# 작을수록 더 부드럽지만 반응은 느려짐
LPF_ALPHA = 0.35

# 하나의 이동을 몇 개 waypoint로 나눌지
LPF_POINTS = 8

# /act/action 발행 주기
ACTION_PUBLISH_HZ = 50.0


KEY_BINDINGS: Dict[str, Tuple[int, float]] = {
    "1": (0, +1.0),
    "q": (0, -1.0),

    "2": (1, +1.0),
    "w": (1, -1.0),

    "3": (2, +1.0),
    "e": (2, -1.0),

    "4": (3, +1.0),
    "r": (3, -1.0),
}


def make_duration(seconds: float) -> Duration:
    msg = Duration()

    whole_seconds = int(seconds)
    nanoseconds = int(
        (seconds - whole_seconds) * 1_000_000_000
    )

    msg.sec = whole_seconds
    msg.nanosec = nanoseconds

    return msg


def clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:
    return max(minimum, min(value, maximum))


class SmoothPickTeleop(Node):
    def __init__(self) -> None:
        super().__init__("smooth_pick_teleop")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            ARM_ACTION_NAME,
        )

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_NAME,
        )

        self.action_publisher = self.create_publisher(
            Float32MultiArray,
            ACT_ACTION_TOPIC,
            10,
        )

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self.joint_state_callback,
            10,
        )

        self.create_timer(
            1.0 / ACTION_PUBLISH_HZ,
            self.publish_action,
        )

        self.latest_arm_qpos: Optional[List[float]] = None
        self.latest_gripper_qpos: Optional[float] = None

        self.desired_arm: Optional[List[float]] = None
        self.current_command_arm: Optional[List[float]] = None
        self.gripper_command: Optional[float] = None

        # 현재 실행 중인 소프트웨어 명령 계획
        self.plan_start_time: Optional[float] = None
        self.plan_times: List[float] = []
        self.plan_positions: List[List[float]] = []

        self.initialized = False

    # --------------------------------------------------------
    # 서버 확인
    # --------------------------------------------------------
    def wait_for_servers(self) -> bool:
        self.get_logger().info("팔 action server 확인 중...")

        if not self.arm_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                f"팔 action server 없음: {ARM_ACTION_NAME}"
            )
            return False

        self.get_logger().info("그리퍼 action server 확인 중...")

        if not self.gripper_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                f"그리퍼 action server 없음: "
                f"{GRIPPER_ACTION_NAME}"
            )
            return False

        return True

    # --------------------------------------------------------
    # 실제 관절 상태 수신
    # --------------------------------------------------------
    def joint_state_callback(
        self,
        msg: JointState,
    ) -> None:
        name_to_index = {
            name: index
            for index, name in enumerate(msg.name)
        }

        required_names = (
            ARM_JOINT_NAMES + ["gripper_left_joint"]
        )

        if not all(
            name in name_to_index
            for name in required_names
        ):
            return

        arm_qpos = [
            float(msg.position[name_to_index[name]])
            for name in ARM_JOINT_NAMES
        ]

        gripper_qpos = float(
            msg.position[
                name_to_index["gripper_left_joint"]
            ]
        )

        self.latest_arm_qpos = arm_qpos
        self.latest_gripper_qpos = gripper_qpos

        if not self.initialized:
            self.desired_arm = arm_qpos.copy()
            self.current_command_arm = arm_qpos.copy()
            self.gripper_command = gripper_qpos
            self.initialized = True

            self.get_logger().info(
                "관절 초기값 수신 완료"
            )

    # --------------------------------------------------------
    # 키 입력 처리
    # --------------------------------------------------------
    def handle_key(self, key: str) -> bool:
        if key == "\x1b":
            # ESC
            return False

        if not self.initialized:
            self.get_logger().warning(
                "/joint_states 초기값을 아직 받지 못했습니다."
            )
            return True

        if key in KEY_BINDINGS:
            joint_index, direction = KEY_BINDINGS[key]

            assert self.desired_arm is not None

            lower, upper = ARM_LIMITS[joint_index]

            safe_lower = lower + JOINT_LIMIT_MARGIN
            safe_upper = upper - JOINT_LIMIT_MARGIN

            previous_value = self.desired_arm[joint_index]

            requested_value = (
                previous_value
                + direction * JOINT_STEP
            )

            new_value = clamp(
                requested_value,
                safe_lower,
                safe_upper,
            )

            self.desired_arm[joint_index] = new_value

            if new_value == previous_value:
                self.get_logger().warning(
                    f"{ARM_JOINT_NAMES[joint_index]} "
                    "소프트웨어 제한 도달"
                )
                return True

            self.send_smoothed_arm_goal(
                path_time=PATH_TIME,
                point_count=LPF_POINTS,
            )

            return True

        if key == "o":
            self.send_gripper_goal(
                G_OPEN,
                "G_OPEN",
            )
            return True

        if key == "p":
            self.send_gripper_goal(
                G_GRASP,
                "G_GRASP",
            )
            return True

        if key == " ":
            self.hold_current_pose()
            return True

        if key == "h":
            print_help()
            return True

        return True

    # --------------------------------------------------------
    # LPF waypoint 생성 후 팔 action 전송
    # --------------------------------------------------------
    def send_smoothed_arm_goal(
        self,
        path_time: float,
        point_count: int,
    ) -> None:
        if (
            self.desired_arm is None
            or self.current_command_arm is None
        ):
            return

        start_position = self.current_command_arm.copy()
        filtered_position = start_position.copy()

        trajectory_points: List[JointTrajectoryPoint] = []

        self.plan_start_time = time.monotonic()
        self.plan_times = [0.0]
        self.plan_positions = [start_position.copy()]

        for point_index in range(1, point_count + 1):
            if point_index == point_count:
                # 마지막 점은 정확히 목표값에 도달
                filtered_position = self.desired_arm.copy()

            else:
                filtered_position = [
                    current
                    + LPF_ALPHA * (target - current)
                    for current, target in zip(
                        filtered_position,
                        self.desired_arm,
                    )
                ]

            point_time = (
                path_time
                * point_index
                / point_count
            )

            point = JointTrajectoryPoint()
            point.positions = filtered_position.copy()
            point.time_from_start = make_duration(
                point_time
            )

            trajectory_points.append(point)

            self.plan_times.append(point_time)
            self.plan_positions.append(
                filtered_position.copy()
            )

        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory.joint_names = (
            ARM_JOINT_NAMES.copy()
        )

        goal_msg.trajectory.points = trajectory_points

        send_future = self.arm_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
            self.arm_goal_response_callback
        )

        rounded = [
            round(value, 4)
            for value in self.desired_arm
        ]

        self.get_logger().info(
            f"팔 목표 전송: {rounded}"
        )

    def arm_goal_response_callback(
        self,
        future,
    ) -> None:
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"팔 goal 전송 예외: {error}"
            )
            return

        if goal_handle is None:
            self.get_logger().error(
                "팔 goal handle 없음"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                "팔 goal 거절됨"
            )

    # --------------------------------------------------------
    # 그리퍼 action 전송
    # --------------------------------------------------------
    def send_gripper_goal(
        self,
        position: float,
        label: str,
    ) -> None:
        goal_msg = GripperCommand.Goal()

        goal_msg.command.position = position
        goal_msg.command.max_effort = (
            GRIPPER_MAX_EFFORT
        )

        self.gripper_command = position

        send_future = self.gripper_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
            lambda future: self.gripper_goal_response_callback(
                future,
                label,
            )
        )

        self.get_logger().info(
            f"그리퍼 목표 전송: "
            f"{label}, position={position}"
        )

    def gripper_goal_response_callback(
        self,
        future,
        label: str,
    ) -> None:
        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"{label} goal 전송 예외: {error}"
            )
            return

        if goal_handle is None:
            self.get_logger().error(
                f"{label} goal handle 없음"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().error(
                f"{label} goal 거절됨"
            )

    # --------------------------------------------------------
    # 현재 실제 자세에서 정지 명령
    # 하드웨어 E-stop이 아니라 소프트웨어 hold임
    # --------------------------------------------------------
    def hold_current_pose(self) -> None:
        if self.latest_arm_qpos is None:
            return

        self.desired_arm = self.latest_arm_qpos.copy()

        self.send_smoothed_arm_goal(
            path_time=0.2,
            point_count=2,
        )

        self.get_logger().warning(
            "현재 실제 관절 위치로 HOLD 명령 전송"
        )

    # --------------------------------------------------------
    # 현재 계획상 명령값 계산
    # --------------------------------------------------------
    def update_current_command(self) -> None:
        if (
            self.plan_start_time is None
            or not self.plan_times
            or not self.plan_positions
        ):
            return

        elapsed = (
            time.monotonic()
            - self.plan_start_time
        )

        if elapsed >= self.plan_times[-1]:
            self.current_command_arm = (
                self.plan_positions[-1].copy()
            )

            self.plan_start_time = None
            self.plan_times = []
            self.plan_positions = []
            return

        for index in range(
            len(self.plan_times) - 1
        ):
            start_time = self.plan_times[index]
            end_time = self.plan_times[index + 1]

            if start_time <= elapsed <= end_time:
                start_position = (
                    self.plan_positions[index]
                )

                end_position = (
                    self.plan_positions[index + 1]
                )

                interval = end_time - start_time

                if interval <= 0.0:
                    ratio = 1.0
                else:
                    ratio = (
                        elapsed - start_time
                    ) / interval

                self.current_command_arm = [
                    start
                    + ratio * (end - start)
                    for start, end in zip(
                        start_position,
                        end_position,
                    )
                ]

                return

    # --------------------------------------------------------
    # Recorder용 실제 action 발행
    # 순서:
    # joint1, joint2, joint3, joint4, gripper
    # --------------------------------------------------------
    def publish_action(self) -> None:
        if not self.initialized:
            return

        if (
            self.current_command_arm is None
            or self.gripper_command is None
        ):
            return

        self.update_current_command()

        msg = Float32MultiArray()

        msg.data = [
            float(value)
            for value in (
                self.current_command_arm
                + [self.gripper_command]
            )
        ]

        self.action_publisher.publish(msg)


def read_key() -> Optional[str]:
    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        0.0,
    )

    if not readable:
        return None

    return sys.stdin.read(1)


def print_help() -> None:
    print(
        "\n"
        "========== Smooth Pick Teleop ==========\n"
        "joint1 : 1 증가 / q 감소\n"
        "joint2 : 2 증가 / w 감소\n"
        "joint3 : 3 증가 / e 감소\n"
        "joint4 : 4 증가 / r 감소\n"
        "\n"
        "o : 그리퍼 열기\n"
        "p : 그리퍼 파지값\n"
        "SPACE : 현재 실제 자세에서 software HOLD\n"
        "h : 도움말 다시 보기\n"
        "ESC 또는 Ctrl+C : 종료\n"
        "========================================\n"
    )


def main() -> None:
    rclpy.init()

    node = SmoothPickTeleop()

    terminal_fd = sys.stdin.fileno()
    old_terminal_settings = termios.tcgetattr(
        terminal_fd
    )

    try:
        if not node.wait_for_servers():
            return

        print("관절 초기값을 기다리는 중...")

        wait_start = time.monotonic()

        while rclpy.ok() and not node.initialized:
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if time.monotonic() - wait_start > 10.0:
                raise RuntimeError(
                    "/joint_states에서 필요한 관절값을 "
                    "10초 안에 받지 못했습니다."
                )

        print_help()

        tty.setcbreak(terminal_fd)

        keep_running = True

        while rclpy.ok() and keep_running:
            rclpy.spin_once(
                node,
                timeout_sec=0.02,
            )

            key = read_key()

            if key is not None:
                keep_running = node.handle_key(key)

    except KeyboardInterrupt:
        print("\n사용자 중단")

    finally:
        termios.tcsetattr(
            terminal_fd,
            termios.TCSADRAIN,
            old_terminal_settings,
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()