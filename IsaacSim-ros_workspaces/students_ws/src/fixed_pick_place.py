#!/usr/bin/env python3

import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand


# ============================================================
# 1. 컨트롤러 action 이름
# ============================================================

ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"


# ============================================================
# 2. 팔 joint 이름
# ros2 param get /arm_controller joints 결과:
# ['joint1', 'joint2', 'joint3', 'joint4']
# ============================================================

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]


# ============================================================
# 3. 팔 자세값
# /joint_states에서 name-position을 매칭해서 기록해야 함.
# 순서 무조건 joint1, joint2, joint3, joint4
# ============================================================

# 현재 hardware.launch.py 실행 후 자동으로 잡힌 안정 자세
# P_HOME: Optional[List[float]] = [
#     -0.0015339807878856412,   # joint1
#     -1.0461748973380072,      # joint2
#     1.0753205323078345,       # joint3
#     0.009203884727313847,     # joint4
# ]

# # 물체 위쪽 자세
# P_PRE_PICK: Optional[List[float]] = [
#     -0.019941750242513337,   # joint1
#     0.8375535101855601,      # joint2
#     -0.12578642460662257,    # joint3
#     -0.8436894333371027,     # joint4
# ]

# # 물체를 실제로 잡는 낮은 자세
# P_PICK: Optional[List[float]] = [
#     -0.02454369260617026,   # joint1
#     1.2885438618239387,     # joint2
#     -0.7731263170943632,    # joint3
#     -0.4816699673960913,    # joint4
# ]

# # 물체를 잡고 위로 들어올린 자세
# P_LIFT: Optional[List[float]] = [
#     -0.02454369260617026,    # joint1
#     0.518485063053467,       # joint2
#     0.04908738521234052,     # joint3
#     -0.6181942575179133,     # joint4
# ]

# # 물체를 든 채 터틀봇이 이동하기 좋은 안전 자세
# P_CARRY: Optional[List[float]] = [
#     -0.039883500485026674,   # joint1
#     -0.9188544919434991,     # joint2
#     0.48013598600820567,     # joint3
#     0.4724660826687775,      # joint4
# ]

# # 현재 hardware.launch.py 실행 후 자동으로 잡힌 안정 자세
# P_HOME: Optional[List[float]] = [
#     -0.0015339807878856412 - 1.5707963268,   # joint1
#     -1.0461748973380072,      # joint2
#     1.0753205323078345,       # joint3
#     0.009203884727313847,     # joint4
# ]

# # 물체 위쪽 자세
# P_PRE_PICK: Optional[List[float]] = [
#     -0.019941750242513337 - 1.5707963268,   # joint1
#     0.8375535101855601,      # joint2
#     -0.12578642460662257,    # joint3
#     -0.8436894333371027,     # joint4
# ]

# # 물체를 실제로 잡는 낮은 자세
# P_PICK: Optional[List[float]] = [
#     -0.02454369260617026 - 1.5707963268,   # joint1
#     1.2885438618239387,     # joint2
#     -0.7731263170943632,    # joint3
#     -0.4816699673960913,    # joint4
# ]

# # 물체를 잡고 위로 들어올린 자세
# P_LIFT: Optional[List[float]] = [
#     -0.02454369260617026 - 1.5707963268,    # joint1
#     0.518485063053467,       # joint2
#     0.04908738521234052,     # joint3
#     -0.6181942575179133,     # joint4
# ]

# # 물체를 든 채 터틀봇이 이동하기 좋은 안전 자세
# P_CARRY: Optional[List[float]] = [
#     -0.039883500485026674 - 1.5707963268,   # joint1
#     -0.9188544919434991,     # joint2
#     0.48013598600820567,     # joint3
#     0.4724660826687775,      # joint4
# ]

# 현재 hardware.launch.py 실행 후 자동으로 잡힌 안정 자세
P_HOME: Optional[List[float]] = [
    -0.0015339807878856412 - 1.5707963268,   # joint1
    -1.0461748973380072,      # joint2
    1.0753205323078345,       # joint3
    0.009203884727313847,     # joint4
]

# 물체 위쪽 자세
P_PRE_PICK: Optional[List[float]] = [
    -0.019941750242513337 - 1.5707963268,   # joint1
    0.8375535101855601,      # joint2
    -0.12578642460662257,    # joint3
    -0.8436894333371027,     # joint4
]

# 물체를 실제로 잡는 낮은 자세
P_PICK: Optional[List[float]] = [
    -0.02454369260617026 - 1.5707963268,   # joint1
    1.2885438618239387,     # joint2
    -0.7731263170943632,    # joint3
    -0.4816699673960913,    # joint4
]

# 물체를 잡고 위로 들어올린 자세
P_LIFT: Optional[List[float]] = [
    -0.02454369260617026 - 1.5707963268,    # joint1
    0.0,       # joint2
    0.0,     # joint3
    0.0,     # joint4
]

# 물체를 든 채 터틀봇이 이동하기 좋은 안전 자세
P_CARRY: Optional[List[float]] = [
    -0.039883500485026674 - 1.5707963268,   # joint1
    0.0,     # joint2
    0.0,     # joint3
    0.0,      # joint4
]

# 놓을 위치 위쪽 자세
P_PRE_PLACE = P_PRE_PICK

# 실제로 물체를 놓는 낮은 자세
P_PLACE = P_PICK


# ============================================================
# 4. 그리퍼 값
# teleop은 open/close만 있지만,
# 중간 파지값은 action에 position 값을 직접 보내서 찾는다.
# ============================================================

# 완전히 열린 값
G_OPEN: Optional[float] = 0.02501155674647538

# 물체 없이 완전히 닫힌 값
# pick에는 직접 쓰지 않음. 기준값으로만 기록.
G_FULL_CLOSE: Optional[float] = -0.0143580601746096

# 물체를 잡을 만큼 닫힌 중간값
# pick에서는 이 값을 사용해야 함.
G_GRASP: Optional[float] = 0.002


# ============================================================
# 5. 이동 시간 / 대기 시간
# 처음에는 천천히.
# ============================================================

ARM_MOVE_TIME = 3.0
GRIPPER_MAX_EFFORT = 0.0
WAIT_AFTER_GRIP = 1.0
LIFT_HOLD_TIME = 3.0


class FixedPickPlace(Node):
    def __init__(self):
        super().__init__("fixed_pick_place")

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

    # --------------------------------------------------------
    # action server 연결 확인
    # --------------------------------------------------------
    def wait_servers(self) -> bool:
        self.get_logger().info(f"팔 action server 대기: {ARM_ACTION}")
        arm_ready = self.arm_client.wait_for_server(timeout_sec=5.0)

        self.get_logger().info(f"그리퍼 action server 대기: {GRIPPER_ACTION}")
        gripper_ready = self.gripper_client.wait_for_server(timeout_sec=5.0)

        if not arm_ready:
            self.get_logger().error(f"팔 action server 없음: {ARM_ACTION}")
            return False

        if not gripper_ready:
            self.get_logger().error(f"그리퍼 action server 없음: {GRIPPER_ACTION}")
            return False

        self.get_logger().info("팔/그리퍼 action server 연결 완료")
        return True

    # --------------------------------------------------------
    # 팔 자세값 검사
    # --------------------------------------------------------
    def check_arm_pose(self, pose: Optional[List[float]], label: str) -> bool:
        if pose is None:
            self.get_logger().error(f"{label} 값이 None임. 아직 자세값을 기록하지 않음.")
            return False

        if len(pose) != len(ARM_JOINT_NAMES):
            self.get_logger().error(
                f"{label} 길이가 joint 개수와 다름. "
                f"pose={len(pose)}, joints={len(ARM_JOINT_NAMES)}"
            )
            return False

        return True

    # --------------------------------------------------------
    # 그리퍼 값 검사
    # --------------------------------------------------------
    def check_gripper_value(self, value: Optional[float], label: str) -> bool:
        if value is None:
            self.get_logger().error(f"{label} 값이 None임. 아직 그리퍼 값을 기록하지 않음.")
            return False
        return True

    # --------------------------------------------------------
    # Duration 메시지 만들기
    # --------------------------------------------------------
    def make_duration(self, seconds: float) -> Duration:
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
        return msg

    # --------------------------------------------------------
    # 팔 action goal 보내기
    # FollowJointTrajectory 사용
    # --------------------------------------------------------
    def send_arm_goal(self, positions: List[float], duration_sec: float, label: str) -> bool:
        self.get_logger().info(f"팔 이동 시작: {label}")
        self.get_logger().info(f"목표값: {positions}")

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = self.make_duration(duration_sec)

        goal_msg.trajectory.points.append(point)

        send_future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"팔 goal 전송 실패: {label}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"팔 goal 거절됨: {label}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrapper = result_future.result()

        if result_wrapper is None:
            self.get_logger().error(f"팔 결과 없음: {label}")
            return False

        status = result_wrapper.status
        result = result_wrapper.result

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"팔 이동 실패: {label}, status={status}, "
                f"error_code={result.error_code}, error_string={result.error_string}"
            )
            return False

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"팔 trajectory 실패: {label}, "
                f"error_code={result.error_code}, error_string={result.error_string}"
            )
            return False

        self.get_logger().info(f"팔 이동 완료: {label}")
        return True

    # --------------------------------------------------------
    # 그리퍼 action goal 보내기
    # GripperCommand 사용
    # --------------------------------------------------------
    def send_gripper_goal(self, position: float, max_effort: float, label: str) -> bool:
        self.get_logger().info(f"그리퍼 동작 시작: {label}, position={position}")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        send_future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"그리퍼 goal 전송 실패: {label}")
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f"그리퍼 goal 거절됨: {label}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrapper = result_future.result()

        if result_wrapper is None:
            self.get_logger().error(f"그리퍼 결과 없음: {label}")
            return False

        status = result_wrapper.status
        result = result_wrapper.result

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"그리퍼 동작 실패: {label}, status={status}")
            return False

        self.get_logger().info(
            f"그리퍼 동작 완료: {label}, "
            f"reached_goal={result.reached_goal}, "
            f"stalled={result.stalled}, "
            f"position={result.position}, "
            f"effort={result.effort}"
        )

        return True

    # --------------------------------------------------------
    # 팔 단계별 실행
    # --------------------------------------------------------
    def run_arm_step(self, pose: Optional[List[float]], label: str) -> bool:
        if not self.check_arm_pose(pose, label):
            return False
        return self.send_arm_goal(pose, ARM_MOVE_TIME, label)

    # --------------------------------------------------------
    # 그리퍼 단계별 실행
    # --------------------------------------------------------
    def run_gripper_step(self, value: Optional[float], label: str) -> bool:
        if not self.check_gripper_value(value, label):
            return False
        return self.send_gripper_goal(value, GRIPPER_MAX_EFFORT, label)

    # --------------------------------------------------------
    # Pick 전체 실행
    # 처음 테스트 때는 바로 실행하지 말 것.
    # 단계별 실행으로 값 검증 후 사용.
    # --------------------------------------------------------
    def run_pick(self) -> bool:
        self.get_logger().info("===== PICK 시작 =====")

        checks = [
            self.check_arm_pose(P_HOME, "P_HOME"),
            self.check_arm_pose(P_PRE_PICK, "P_PRE_PICK"),
            self.check_arm_pose(P_PICK, "P_PICK"),
            self.check_arm_pose(P_LIFT, "P_LIFT"),
            self.check_arm_pose(P_CARRY, "P_CARRY"),
            self.check_gripper_value(G_OPEN, "G_OPEN"),
            self.check_gripper_value(G_GRASP, "G_GRASP"),
        ]

        if not all(checks):
            self.get_logger().error("PICK에 필요한 값이 부족함. 실행 중단.")
            return False

        steps = [
            ("gripper_open", lambda: self.send_gripper_goal(G_OPEN, GRIPPER_MAX_EFFORT, "G_OPEN")),
            ("home", lambda: self.send_arm_goal(P_HOME, ARM_MOVE_TIME, "P_HOME")),
            ("pre_pick", lambda: self.send_arm_goal(P_PRE_PICK, ARM_MOVE_TIME, "P_PRE_PICK")),
            ("pick", lambda: self.send_arm_goal(P_PICK, ARM_MOVE_TIME, "P_PICK")),
            ("gripper_grasp", lambda: self.send_gripper_goal(G_GRASP, GRIPPER_MAX_EFFORT, "G_GRASP")),
            ("lift", lambda: self.send_arm_goal(P_LIFT, ARM_MOVE_TIME, "P_LIFT")),
            ("carry", lambda: self.send_arm_goal(P_CARRY, ARM_MOVE_TIME, "P_CARRY")),
        ]

        for step_name, step_func in steps:
            self.get_logger().info(f"PICK 단계 실행: {step_name}")
            ok = step_func()

            if not ok:
                self.get_logger().error(f"PICK 실패 단계: {step_name}")
                return False

            if step_name == "gripper_grasp":
                time.sleep(WAIT_AFTER_GRIP)

            if step_name == "lift":
                time.sleep(LIFT_HOLD_TIME)

        self.get_logger().info("===== PICK 성공 =====")
        return True

    # --------------------------------------------------------
    # Place 전체 실행
    # --------------------------------------------------------
    def run_place(self) -> bool:
        self.get_logger().info("===== PLACE 시작 =====")

        checks = [
            self.check_arm_pose(P_PRE_PLACE, "P_PRE_PLACE"),
            self.check_arm_pose(P_PLACE, "P_PLACE"),
            self.check_arm_pose(P_HOME, "P_HOME"),
            self.check_gripper_value(G_OPEN, "G_OPEN"),
        ]

        if not all(checks):
            self.get_logger().error("PLACE에 필요한 값이 부족함. 실행 중단.")
            return False

        steps = [
            ("pre_place", lambda: self.send_arm_goal(P_PRE_PLACE, ARM_MOVE_TIME, "P_PRE_PLACE")),
            ("place", lambda: self.send_arm_goal(P_PLACE, ARM_MOVE_TIME, "P_PLACE")),
            ("gripper_open", lambda: self.send_gripper_goal(G_OPEN, GRIPPER_MAX_EFFORT, "G_OPEN")),
            ("back_to_pre_place", lambda: self.send_arm_goal(P_PRE_PLACE, ARM_MOVE_TIME, "P_PRE_PLACE")),
            ("home", lambda: self.send_arm_goal(P_HOME, ARM_MOVE_TIME, "P_HOME")),
        ]

        for step_name, step_func in steps:
            self.get_logger().info(f"PLACE 단계 실행: {step_name}")
            ok = step_func()

            if not ok:
                self.get_logger().error(f"PLACE 실패 단계: {step_name}")
                return False

            if step_name == "gripper_open":
                time.sleep(WAIT_AFTER_GRIP)

        self.get_logger().info("===== PLACE 성공 =====")
        return True


def print_usage():
    print()
    print("사용법:")
    print("  python3 ~/fixed_pick_place.py arm_home")
    print("  python3 ~/fixed_pick_place.py arm_pre_pick")
    print("  python3 ~/fixed_pick_place.py arm_pick")
    print("  python3 ~/fixed_pick_place.py arm_lift")
    print("  python3 ~/fixed_pick_place.py arm_carry")
    print("  python3 ~/fixed_pick_place.py arm_pre_place")
    print("  python3 ~/fixed_pick_place.py arm_place")
    print()
    print("  python3 ~/fixed_pick_place.py gripper_open")
    print("  python3 ~/fixed_pick_place.py gripper_full_close")
    print("  python3 ~/fixed_pick_place.py gripper_grasp")
    print()
    print("  python3 ~/fixed_pick_place.py pick")
    print("  python3 ~/fixed_pick_place.py place")
    print()
    print("주의:")
    print("  - hardware.launch.py가 먼저 켜져 있어야 함")
    print("  - 처음에는 pick/place 전체 실행 금지")
    print("  - 반드시 단계별 실행으로 자세값을 하나씩 확인")
    print("  - G_FULL_CLOSE는 기준값이고, pick에는 G_GRASP를 사용")
    print()


def main():
    if len(sys.argv) != 2:
        print_usage()
        return

    mode = sys.argv[1].lower()

    valid_modes = [
        "arm_home",
        "arm_pre_pick",
        "arm_pick",
        "arm_lift",
        "arm_carry",
        "arm_pre_place",
        "arm_place",
        "gripper_open",
        "gripper_full_close",
        "gripper_grasp",
        "pick",
        "place",
    ]

    if mode not in valid_modes:
        print_usage()
        return

    rclpy.init()
    node = FixedPickPlace()

    try:
        if not node.wait_servers():
            return

        if mode == "arm_home":
            ok = node.run_arm_step(P_HOME, "P_HOME")

        elif mode == "arm_pre_pick":
            ok = node.run_arm_step(P_PRE_PICK, "P_PRE_PICK")

        elif mode == "arm_pick":
            ok = node.run_arm_step(P_PICK, "P_PICK")

        elif mode == "arm_lift":
            ok = node.run_arm_step(P_LIFT, "P_LIFT")

        elif mode == "arm_carry":
            ok = node.run_arm_step(P_CARRY, "P_CARRY")

        elif mode == "arm_pre_place":
            ok = node.run_arm_step(P_PRE_PLACE, "P_PRE_PLACE")

        elif mode == "arm_place":
            ok = node.run_arm_step(P_PLACE, "P_PLACE")

        elif mode == "gripper_open":
            ok = node.run_gripper_step(G_OPEN, "G_OPEN")

        elif mode == "gripper_full_close":
            ok = node.run_gripper_step(G_FULL_CLOSE, "G_FULL_CLOSE")

        elif mode == "gripper_grasp":
            ok = node.run_gripper_step(G_GRASP, "G_GRASP")

        elif mode == "pick":
            ok = node.run_pick()

        elif mode == "place":
            ok = node.run_place()

        else:
            ok = False

        if ok:
            node.get_logger().info(f"{mode} 성공")
        else:
            node.get_logger().error(f"{mode} 실패")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
