# P_REAR_CARRY → P_REAR_PLACE → G_OPEN
# P_REAR_PRE_SWEEP → P_SIDE_CARRY -> P_PUSH
# 터틀봇 전진 → 장애물 밀기 → 정지
# P_REAR_PRE_SWEEP → P_SIDE_CARRY -> P_REAR_PICK
# G_GRASP → P_REAR_CARRY


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


# ROS 2 인터페이스
ARM_ACTION = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/gripper_controller/gripper_cmd"

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
]


# 동작 시간
ARM_MOVE_TIME = 2.0
ARM_ROTATE_TIME = 2.0
WAIT_AFTER_GRIP = 1.0

# 그리퍼 설정
G_OPEN = 0.02501155674647538
G_GRASP = 0.002
GRIPPER_MAX_EFFORT = 0.0

# 시작 자세 확인 허용 오차
START_TOLERANCE = 0.12


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


# 후방 회전 준비 자세
P_REAR_PRE_SWEEP = [
    -3.0234761329225988,
    -0.11044661672776616,
    -0.0030679615757712823,
     0.2193592526676467,
]


# 측면 경유 자세
P_SIDE_CARRY = [
    -1.59073807703741,
    -0.035281558121369745,
    -0.13959225169759334,
     0.2945243112740431,
]


# 전방 Push 자세
P_PUSH = [
    -0.0046019423636569235,
     0.6519418348513975,
    -0.17947575218262002,
    -0.48933987133551954,
]

P_PRE_PUSH = P_PUSH


class FrontPush(Node):

    def __init__(self):

        super().__init__("front_push")

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

    # 실제 관절 위치 수신
    def joint_state_callback(self, msg):

        if not all(name in msg.name for name in JOINT_NAMES):
            return

        if len(msg.position) < len(msg.name):
            return

        self.current_pose = [
            float(msg.position[msg.name.index(name)])
            for name in JOINT_NAMES
        ]

    # Action Server 연결 확인
    def wait_servers(self):

        if not self.arm_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Arm Action Server를 찾을 수 없습니다."
            )
            return False

        if not self.gripper_client.wait_for_server(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                "Gripper Action Server를 찾을 수 없습니다."
            )
            return False

        return True

    # 현재 관절 자세 확인
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

    # 로봇팔 이동
    def move_arm(self, pose, label, duration=None):

        if duration is None:
            duration = ARM_MOVE_TIME

        self.get_logger().info(
            f"팔 이동 시작: {label}, 이동 시간={duration}초"
        )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()

        point.positions = pose.copy()

        point.time_from_start = self.make_duration(
            duration
        )

        goal.trajectory.points = [point]

        send_future = self.arm_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=10.0,
        )

        if not send_future.done():

            self.get_logger().error(
                f"{label}: 명령 전송 시간 초과"
            )
            return False

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
            timeout_sec=duration + 10.0,
        )

        if not result_future.done():

            self.get_logger().error(
                f"{label}: 실행 결과 대기 시간 초과"
            )

            self.get_logger().error(
                "실제 로봇팔 상태를 확인하세요."
            )

            return False

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
                f"status={result.status}, "
                f"error_code={result.result.error_code}"
            )

            return False

        self.get_logger().info(
            f"{label}: 이동 완료"
        )

        return True

    # 그리퍼 열기 및 닫기
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
            timeout_sec=10.0,
        )

        if not send_future.done():

            self.get_logger().error(
                f"{label}: 명령 전송 시간 초과"
            )
            return False

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
            timeout_sec=15.0,
        )

        if not result_future.done():

            self.get_logger().error(
                f"{label}: 결과 대기 시간 초과"
            )
            return False

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
                "그리퍼 열기 목표에 도달하지 못했습니다."
            )
            return False

        return True

    # 후방 Place 후 전방 Push 자세로 이동
    def run_prepare(self):

        self.get_logger().info(
            "===== FRONT PUSH PREPARE START ====="
        )

        # 후방 Carry에서 시작하는지 확인
        if not self.check_pose(
            P_REAR_CARRY,
            "P_REAR_CARRY",
        ):
            return False

        # 배달 물체 내려놓기
        if not self.move_arm(
            P_REAR_PLACE,
            "P_REAR_PLACE",
        ):
            return False

        if not self.move_gripper(
            G_OPEN,
            "G_OPEN",
        ):
            return False

        time.sleep(WAIT_AFTER_GRIP)

        # 후방에서 물체와 간섭하지 않도록 팔 들어 올리기
        if not self.move_arm(
            P_REAR_PRE_SWEEP,
            "P_REAR_PRE_SWEEP",
        ):
            return False

        # 측면을 거쳐 전방으로 회전
        if not self.move_arm(
            P_SIDE_CARRY,
            "P_SIDE_CARRY",
            duration=ARM_ROTATE_TIME,
        ):
            return False

        # 전방 Push 자세로 이동
        if not self.move_arm(
            P_PUSH,
            "P_PUSH",
            duration=ARM_ROTATE_TIME,
        ):
            return False

        self.get_logger().info(
            "===== FRONT PUSH READY ====="
        )

        self.get_logger().info(
            "네비게이션 팀의 Push 주행을 기다립니다."
        )

        return True

    # Push 주행 및 원위치 복귀 후 배달 물체 재파지
    def run_recover(self):

        self.get_logger().info(
            "===== FRONT PUSH RECOVER START ====="
        )

        # Push 자세에서 시작하는지 확인
        if not self.check_pose(
            P_PUSH,
            "P_PUSH",
        ):
            return False

        # 측면 경유 자세로 복귀
        if not self.move_arm(
            P_SIDE_CARRY,
            "P_SIDE_CARRY",
            duration=ARM_ROTATE_TIME,
        ):
            return False

        # 후방 Pick 준비 자세로 이동
        if not self.move_arm(
            P_REAR_PRE_SWEEP,
            "P_REAR_PRE_SWEEP",
            duration=ARM_ROTATE_TIME,
        ):
            return False

        # 그리퍼 열림 상태 확보
        if not self.move_gripper(
            G_OPEN,
            "G_OPEN",
        ):
            return False

        # 뒤쪽 물체에 접근
        if not self.move_arm(
            P_REAR_PICK,
            "P_REAR_PICK",
        ):
            return False

        # 배달 물체 다시 집기
        if not self.move_gripper(
            G_GRASP,
            "G_GRASP",
        ):
            return False

        time.sleep(WAIT_AFTER_GRIP)

        # 후방 운반 자세로 들어 올리기
        if not self.move_arm(
            P_REAR_CARRY,
            "P_REAR_CARRY",
        ):
            return False

        self.get_logger().info(
            "===== FRONT PUSH RECOVER COMPLETE ====="
        )

        return True


def main():

    if (
        len(sys.argv) != 2
        or sys.argv[1] not in ["prepare", "recover"]
    ):

        print("사용법:")
        print("python3 front_push.py prepare")
        print("python3 front_push.py recover")

        return 2

    mode = sys.argv[1]

    rclpy.init()

    node = FrontPush()

    success = False

    try:

        if node.wait_servers():

            if mode == "prepare":
                success = node.run_prepare()

            elif mode == "recover":
                success = node.run_recover()

        if not success:

            node.get_logger().error(
                "FRONT PUSH 동작 실패 또는 중단"
            )

    except KeyboardInterrupt:

        node.get_logger().warning(
            "사용자가 동작을 중단했습니다."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())