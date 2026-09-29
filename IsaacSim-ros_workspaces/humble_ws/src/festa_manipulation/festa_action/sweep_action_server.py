#!/usr/bin/env python3

import os
import sys
import time
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from turtlebot3_msgs.action import Sweep


# 기존 장애물 제거 시퀀스 (이 파일과 같은 폴더의 복사본을 실행)
SEQUENCE_SCRIPT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "obstacle_clear_sequence.py",
)


class SweepActionServer(Node):

    def __init__(self):
        super().__init__("sweep_action_server")

        # False: obstacle_clear_sequence.py를 --no-safety로 실행
        # (Isaac Sim처럼 전류 토픽이 없는 환경. 실제 로봇은 기본값 True 유지)
        self.safety_monitor = bool(
            self.declare_parameter("safety_monitor", True).value
        )

        # Action 실행 중에도 /sweep_stage 콜백을 받을 수 있도록 설정
        self.callback_group = ReentrantCallbackGroup()

        # obstacle_clear_sequence.py에서 발행하는 현재 동작 단계 수신
        self.stage_sub = self.create_subscription(
            String,
            "/sweep_stage",
            self.stage_callback,
            10,
            callback_group=self.callback_group,
        )

        # Navigation에서 호출할 Sweep Action Server
        self.action_server = ActionServer(
            self,
            Sweep,
            "/sweep",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # 현재 실행 중인 Action Goal
        self.current_goal = None

        # obstacle_clear_sequence.py에서 받은 마지막 종료 상태
        self.final_state = None

        # SEQUENCE_COMPLETE / SEQUENCE_FAILED 수신 확인용
        self.terminal_event = threading.Event()

        # 동시에 두 개의 Sweep이 실행되는 것을 방지
        self.busy = False
        self.busy_lock = threading.Lock()

        self.get_logger().info(
            f"Sweep Action Server ready (safety_monitor={self.safety_monitor})"
        )


    def goal_callback(self, goal_request):
        """
        Navigation에서 새로운 Sweep 요청이 들어왔을 때 호출된다.

        이미 Sweep 동작이 실행 중이면 새로운 요청은 거절한다.
        """

        with self.busy_lock:

            if self.busy:
                self.get_logger().warning(
                    "Sweep 실행 중이므로 새로운 요청을 거절합니다."
                )
                return GoalResponse.REJECT

            self.busy = True

        self.get_logger().info("Sweep Goal 승인")
        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):
        """
        현재 obstacle_clear_sequence.py에는
        외부 Action Cancel과 연동된 안전 정지 절차가 없으므로
        Action Cancel 요청은 지원하지 않는다.
        """

        self.get_logger().warning(
            "현재 Sweep Action Cancel은 지원하지 않습니다."
        )

        return CancelResponse.REJECT


    def stage_callback(self, msg):
        """
        obstacle_clear_sequence.py의 /sweep_stage를 수신한다.

        받은 state는 그대로 Navigation 측 Action Client에
        Feedback으로 전달한다.
        """

        state = msg.data

        self.get_logger().info(f"STATE -> {state}")

        # Navigation으로 현재 동작 단계 전달
        if self.current_goal is not None:

            feedback = Sweep.Feedback()
            feedback.state = state

            self.current_goal.publish_feedback(feedback)

        # 전체 Sequence 종료 상태 기록
        if state in (
            "SEQUENCE_COMPLETE",
            "SEQUENCE_FAILED",
        ):
            self.final_state = state
            self.terminal_event.set()


    def execute_callback(self, goal_handle):
        """
        Navigation의 /sweep 요청을 받으면
        obstacle_clear_sequence.py all 을 실행한다.

        Result:
        SUCCESS
            - 전체 장애물 제거 Sequence 정상 완료

        ERROR
            - Sequence 실패
            - 임시 안전 컷오프
            - 센서 이상
            - 자세 이동 실패
            - 그리퍼 실패
            - 기타 실행 오류

        현재 obstacle_clear_sequence.py에서는
        임시 안전 컷오프와 센서 이상을 Action 수준에서
        정확히 구분할 수 없으므로 STALL은 반환하지 않는다.
        """

        self.get_logger().info(
            "Navigation -> Sweep 요청 수신"
        )

        self.current_goal = goal_handle
        self.final_state = None
        self.terminal_event.clear()

        result = Sweep.Result()

        try:

            # 실행 파일 존재 확인
            if not os.path.isfile(SEQUENCE_SCRIPT):
                raise FileNotFoundError(
                    f"파일을 찾을 수 없습니다: {SEQUENCE_SCRIPT}"
                )

            # 기존 검증된 전체 장애물 제거 시퀀스 실행
            command = [
                sys.executable,
                SEQUENCE_SCRIPT,
                "all",
            ]

            if not self.safety_monitor:
                command.append("--no-safety")

            process = subprocess.Popen(command)

            # obstacle_clear_sequence.py가 종료될 때까지 대기
            while rclpy.ok():

                if process.poll() is not None:
                    break

                time.sleep(0.1)

            # subprocess 종료 직후 마지막 /sweep_stage 메시지가
            # 아직 전달 중일 수 있으므로 잠시 대기
            if not self.terminal_event.is_set():
                self.terminal_event.wait(timeout=1.0)

            # 최종 결과 판정
            if self.final_state == "SEQUENCE_COMPLETE":

                goal_handle.succeed()
                result.result_code = "SUCCESS"

            else:

                goal_handle.abort()
                result.result_code = "ERROR"

        except Exception as e:

            self.get_logger().error(
                f"Sweep Action ERROR: {e}"
            )

            goal_handle.abort()
            result.result_code = "ERROR"

        finally:

            self.current_goal = None

            with self.busy_lock:
                self.busy = False

        self.get_logger().info(
            f"Result -> {result.result_code}"
        )

        return result


def main(args=None):

    rclpy.init(args=args)

    node = SweepActionServer()

    # Action 실행과 /sweep_stage 수신을 동시에 처리
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()