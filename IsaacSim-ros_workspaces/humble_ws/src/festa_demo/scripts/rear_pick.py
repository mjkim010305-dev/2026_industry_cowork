# G_OPEN → P_HOME -> P_PRE_PICK → P_PICK -> G_GRASP → P_LIFT 
# -> P_LIFT_FOLD -> P_SIDE_CARRY -> P_REAR_CARRY



#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState


# ROS 2 Action 이름
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]


# 동작 설정
ARM_MOVE_TIME = 4.0
GRIPPER_MAX_EFFORT = 0.0
WAIT_AFTER_GRIP = 1.0

G_OPEN = 0.02501155674647538
G_GRASP = 0.002

START_TOLERANCE = 0.12


# 기존 Home 자세
P_HOME = [
    -0.0015339807878856412,
    -1.0461748973380072,
     1.0753205323078345,
     0.009203884727313847,
]


# 새로 측정한 전방 Pick 준비 자세
# festa_demo (2026-10-02, user: grasp level, not tilted): same gripper tip point as the
# measured pose (0.334 m ahead, 0.011 m below joint2), gripper pitch 0 instead of 29 deg
# (OpenManipulator-X planar IK). Measured pose was [-0.00307, 0.7271, -0.4801, 0.2562].
P_PRE_PICK = [
    -0.0030679615757712823,
     0.8443,
    -0.1591,
    -0.6852,
]


# 새로 측정한 전방 Pick 자세
# festa_demo (2026-10-02, user): level grasp, same tip point (0.336 m ahead, 0.057 m below
# joint2), pitch 0 instead of 35 deg. Measured pose was [-0.00307, 0.9235, -0.5706, 0.2577].
P_PICK = [
    -0.0030679615757712823,
     1.1229,
    -0.3009,
    -0.8220,
]


# 새로 측정한 Lift 자세
P_LIFT = [
    -0.0030679615757712823,
    -0.04295146206079795,
    -0.13959225169759334,
     0.2945243112740431,
]


# festa_demo (2026-10-02, user: the turn to the rear swung too wide): fold the arm to the
# rear-carry shape while still facing forward, then turn joint1 only. Tip reach during the
# turn 0.14 m instead of 0.27 m (P_LIFT).
P_LIFT_FOLD = [
    -0.0030679615757712823,
    -0.9802137234589247,
     0.5016117176386047,
     0.2193592526676467,
]


# 새로 측정한 측면 Carry 자세 (festa_demo: folded as P_REAR_CARRY; measured pose was
# [-1.5907, -0.0353, -0.1396, 0.2945])
P_SIDE_CARRY = [
    -1.59073807703741,
    -0.9802137234589247,
     0.5016117176386047,
     0.2193592526676467,
]


# 기존에 기록한 후방 Carry 자세 재사용
P_REAR_CARRY = [
    -3.0234761329225988,
    -0.9802137234589247,
     0.5016117176386047,
     0.2193592526676467,
]


class RearPick(Node):

    def __init__(self):
        super().__init__("rear_pick")

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

    # 실제 관절 각도 수신
    def joint_state_callback(self, msg):

        if not all(name in msg.name for name in JOINT_NAMES):
            return

        self.current_pose = [
            float(msg.position[msg.name.index(name)])
            for name in JOINT_NAMES
        ]

    # 팔과 그리퍼 Action Server 확인
    def wait_servers(self):

        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Arm Action Server 없음")
            return False

        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Gripper Action Server 없음")
            return False

        return True

    # 현재 자세가 Home인지 확인
    def check_start_pose(self):

        self.current_pose = None

        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_pose is not None:
                break

        if self.current_pose is None:
            self.get_logger().error("/joint_states 수신 실패")
            return False

        errors = [
            abs(current - target)
            for current, target in zip(
                self.current_pose,
                P_HOME,
            )
        ]

        if max(errors) > START_TOLERANCE:

            self.get_logger().error(
                "현재 자세가 P_HOME과 다릅니다."
            )

            self.get_logger().error(
                f"현재 관절값: {self.current_pose}"
            )

            return False

        self.get_logger().info("P_HOME 확인 완료")
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

    # 로봇팔 목표 자세로 이동
    def move_arm(self, pose, label):

        self.get_logger().info(f"팔 이동 시작: {label}")

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
                f"{label}: 명령 거절"
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
                f"{label}: 이동 실패, "
                f"error_code={result.result.error_code}"
            )
            return False

        self.get_logger().info(f"{label}: 이동 완료")
        return True

    # 그리퍼 열기 또는 닫기
    def move_gripper(self, position, label):

        self.get_logger().info(
            f"그리퍼 동작 시작: {label}"
        )

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        send_future = self.gripper_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"{label}: 명령 거절"
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

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"{label}: 그리퍼 동작 실패"
            )
            return False

        grip_result = result.result

        self.get_logger().info(
            f"{label} 완료: "
            f"position={grip_result.position}, "
            f"reached_goal={grip_result.reached_goal}, "
            f"stalled={grip_result.stalled}"
        )

        if label == "G_OPEN" and not grip_result.reached_goal:
            self.get_logger().error(
                "그리퍼가 열린 목표 위치에 도달하지 못했습니다."
            )
            return False

        return True

    # 전방 Pick 후 후방 Carry까지 실행

    # 전방 Pick 후 후방 Carry까지 자동 실행
    def run_pick(self):

        self.get_logger().info("===== REAR PICK START =====")

        # STEP 1. Home 자세 확인 및 그리퍼 열기
        if not self.check_start_pose():
            return False

        if not self.move_gripper(G_OPEN, "G_OPEN"):
            return False

        # STEP 2. 전방 물체 접근
        if not self.move_arm(P_PRE_PICK, "P_PRE_PICK"):
            return False

        if not self.move_arm(P_PICK, "P_PICK"):
            return False

        # STEP 3. 물체 파지 및 들어 올리기
        if not self.move_gripper(G_GRASP, "G_GRASP"):
            return False

        time.sleep(WAIT_AFTER_GRIP)

        if not self.move_arm(P_LIFT, "P_LIFT"):
            return False

        # STEP 4. 접은 뒤 측면 Carry 자세로 이동
        if not self.move_arm(P_LIFT_FOLD, "P_LIFT_FOLD"):
            return False

        if not self.move_arm(P_SIDE_CARRY, "P_SIDE_CARRY"):
            return False

        # STEP 5. 후방 Carry 자세로 이동
        if not self.move_arm(P_REAR_CARRY, "P_REAR_CARRY"):
            return False

        self.get_logger().info("===== REAR PICK COMPLETE =====")

        return True


def main():

    if len(sys.argv) != 2 or sys.argv[1] != "pick":
        print("사용법: python3 rear_pick.py pick")
        return

    rclpy.init()

    node = RearPick()

    try:

        if not node.wait_servers():
            return

        if not node.run_pick():
            node.get_logger().error(
                "REAR PICK 실패"
            )

    except KeyboardInterrupt:

        node.get_logger().warning(
            "사용자가 동작을 중단했습니다."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()