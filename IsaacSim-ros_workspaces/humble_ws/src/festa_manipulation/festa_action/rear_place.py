# P_REAR_CARRY -> P_REAR_PLACE = P_REAR_PICK -> G_OPEN -> 
# P_REAR_PRE_SWEEP -> P_SIDE_PRE_SWEEP -> P_PRE_SWEEP


#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


# ============================================================
# 1. ROS interfaces
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
# 2. 동작 설정
# ============================================================

# 최초 실험에서는 천천히 이동
ARM_MOVE_TIME = 4.0

# 기존 성공 코드의 그리퍼 설정
G_OPEN = 0.02501155674647538
GRIPPER_MAX_EFFORT = 0.0

# 시작 자세 허용 오차 (rad)
START_TOLERANCE = 0.12


# ============================================================
# 3. 후방 자세
# 순서: joint1, joint2, joint3, joint4
# ============================================================

# STEP 1. 뒤쪽에서 물체를 들고 있는 시작 자세
P_REAR_CARRY = [
    -3.0234761329225988,
    -0.9802137234589247,
     0.5016117176386047,
     0.2193592526676467,
]


# STEP 2. 뒤쪽 물체 내려놓기
# 기존 P_REAR_PICK 자세와 동일
P_REAR_PLACE = [
    -3.0234761329225988,
     0.679553489033339,
    -0.19634954084936207,
     0.2193592526676467,
]


# ============================================================
# 4. Rear Place Node
# ============================================================

class RearPlace(Node):

    def __init__(self):
        super().__init__("rear_place")

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

        self.current_pose = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    # --------------------------------------------------------
    # 실제 관절값 수신
    # --------------------------------------------------------

    def joint_state_callback(self, msg):

        if not all(name in msg.name for name in JOINT_NAMES):
            return

        self.current_pose = [
            float(msg.position[msg.name.index(name)])
            for name in JOINT_NAMES
        ]

    # --------------------------------------------------------
    # Action Server 연결 확인
    # --------------------------------------------------------

    def wait_servers(self):

        if not self.arm_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Arm Action Server 없음"
            )
            return False

        if not self.gripper_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Gripper Action Server 없음"
            )
            return False

        return True

    # --------------------------------------------------------
    # STEP 1. 시작 자세 확인
    # 실제 관절값을 확인하며 팔은 움직이지 않음
    # --------------------------------------------------------

    def check_start_pose(self):

        self.get_logger().info(
            "STEP 1: P_REAR_CARRY 확인"
        )

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
                "현재 관절값을 받지 못했습니다."
            )
            return False

        errors = [
            abs(current - target)
            for current, target in zip(
                self.current_pose,
                P_REAR_CARRY,
            )
        ]

        if max(errors) > START_TOLERANCE:

            self.get_logger().error(
                "현재 자세가 P_REAR_CARRY와 다릅니다."
            )

            self.get_logger().error(
                f"현재 관절값: {self.current_pose}"
            )

            return False

        self.get_logger().info(
            "P_REAR_CARRY 확인 완료"
        )

        return True

    # --------------------------------------------------------
    # STEP 2. 후방 Place 자세로 이동
    # --------------------------------------------------------

    @staticmethod
    def make_duration(seconds):

        sec = int(seconds)

        return Duration(
            sec=sec,
            nanosec=int(
                (seconds - sec) * 1_000_000_000
            ),
        )

    def move_arm(self, pose, label):

        self.get_logger().info(
            f"STEP 2: {label} 이동 시작"
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()

        point.positions = pose
        point.time_from_start = self.make_duration(
            ARM_MOVE_TIME
        )

        goal.trajectory.points = [point]

        send_future = self.arm_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"{label}: 목표 명령 거절"
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                f"{label}: 결과 수신 실패"
            )
            return False

        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error(
                f"{label}: 이동 실패"
            )
            return False

        self.get_logger().info(
            f"{label}: 이동 완료"
        )

        return True

    # --------------------------------------------------------
    # STEP 3. 그리퍼 열기
    # --------------------------------------------------------

    def open_gripper(self):

        self.get_logger().info(
            "STEP 3: G_OPEN"
        )

        goal = GripperCommand.Goal()

        goal.command.position = G_OPEN
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        send_future = self.gripper_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                "그리퍼 명령 거절"
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result = result_future.result()

        if result is None:
            self.get_logger().error(
                "그리퍼 결과 수신 실패"
            )
            return False

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                "그리퍼 열기 실패"
            )
            return False

        if not result.result.reached_goal:
            self.get_logger().error(
                "그리퍼가 열린 목표 위치에 도달하지 못했습니다."
            )
            return False

        self.get_logger().info(
            "G_OPEN 완료"
        )

        return True

    # --------------------------------------------------------
    # Rear Place 전체 실행
    # --------------------------------------------------------

    def run(self):

        self.get_logger().info(
            "===== REAR PLACE START ====="
        )

        # STEP 1
        if not self.check_start_pose():
            return False

        # STEP 2
        if not self.move_arm(
            P_REAR_PLACE,
            "P_REAR_PLACE",
        ):
            return False

        # STEP 3
        if not self.open_gripper():
            return False

        self.get_logger().info(
            "===== REAR PLACE COMPLETE ====="
        )

        return True


# ============================================================
# Main
# ============================================================

def main():

    if len(sys.argv) != 2 or sys.argv[1] != "place":
        print("사용법: python3 rear_place.py place")
        return

    rclpy.init()

    node = RearPlace()

    try:

        if not node.wait_servers():
            return

        if not node.run():
            node.get_logger().error(
                "REAR PLACE 실패"
            )

    except KeyboardInterrupt:

        node.get_logger().warning(
            "사용자가 실행을 중단했습니다."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()