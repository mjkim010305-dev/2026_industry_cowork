#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint


# ============================================================
# ROS interfaces
# ============================================================

ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]


# ============================================================
# Reset configuration
# ============================================================

ARM_MOVE_TIME = 4.0

# 기존 fixed_pick_place.py에서 실제 사용한 HOME
P_HOME = [
    -0.0015339807878856412,
    -1.0461748973380072,
    1.0753205323078345,
    0.009203884727313847,
]

# 기존 fixed_pick_place.py의 실제 OPEN 값
G_OPEN = 0.02501155674647538


class ResetHome(Node):

    def __init__(self):
        super().__init__("reset_home")

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
    # Wait for action servers
    # --------------------------------------------------------

    def wait_servers(self) -> bool:

        self.get_logger().info(
            f"Waiting for gripper action server: {GRIPPER_ACTION}"
        )

        if not self.gripper_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Gripper action server를 찾을 수 없습니다."
            )
            return False

        self.get_logger().info(
            f"Waiting for arm action server: {ARM_ACTION}"
        )

        if not self.arm_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Arm action server를 찾을 수 없습니다."
            )
            return False

        return True

    # --------------------------------------------------------
    # Duration helper
    # --------------------------------------------------------

    @staticmethod
    def make_duration(seconds: float) -> Duration:

        sec = int(seconds)
        nanosec = int(
            (seconds - sec) * 1_000_000_000
        )

        return Duration(
            sec=sec,
            nanosec=nanosec,
        )

    # --------------------------------------------------------
    # Open gripper
    # --------------------------------------------------------

    def open_gripper(self) -> bool:

        self.get_logger().info(
            f"GRIPPER OPEN -> {G_OPEN}"
        )

        goal = GripperCommand.Goal()
        goal.command.position = G_OPEN

        send_future = self.gripper_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            self.get_logger().error(
                "Gripper OPEN goal rejected"
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        if result_future.result() is None:
            self.get_logger().error(
                "Gripper OPEN result를 받지 못했습니다."
            )
            return False

        self.get_logger().info(
            "GRIPPER OPEN 완료"
        )

        return True

    # --------------------------------------------------------
    # Move arm HOME
    # --------------------------------------------------------

    def move_home(self) -> bool:

        self.get_logger().info(
            "Moving -> P_HOME"
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = (
            JOINT_NAMES
        )

        point = JointTrajectoryPoint()

        point.positions = P_HOME

        point.time_from_start = (
            self.make_duration(
                ARM_MOVE_TIME
            )
        )

        goal.trajectory.points = [
            point
        ]

        send_future = (
            self.arm_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            self.get_logger().error(
                "P_HOME goal rejected"
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                "P_HOME result를 받지 못했습니다."
            )
            return False

        error_code = (
            result.result.error_code
        )

        if error_code != 0:
            self.get_logger().error(
                f"P_HOME failed, "
                f"error_code={error_code}"
            )
            return False

        self.get_logger().info(
            "P_HOME 완료"
        )

        return True

    # --------------------------------------------------------
    # Reset sequence
    # --------------------------------------------------------

    def run_reset(self) -> bool:

        # 반드시 그리퍼 먼저 OPEN
        if not self.open_gripper():
            return False

        # 그 다음 HOME
        if not self.move_home():
            return False

        self.get_logger().info(
            "RESET COMPLETE: "
            "GRIPPER OPEN -> P_HOME"
        )

        return True


def main():

    rclpy.init()

    node = ResetHome()

    try:

        if not node.wait_servers():
            return

        node.run_reset()

    except KeyboardInterrupt:

        node.get_logger().warning(
            "사용자가 중단했습니다."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
