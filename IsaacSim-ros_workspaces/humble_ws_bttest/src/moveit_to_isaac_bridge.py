#!/usr/bin/env python3
"""
MoveIt <-> Isaac Sim bridge for the TurtleBot3 Manipulation (OpenMANIPULATOR-X) arm.

move_group.launch.py 는 `move_group` 노드만 띄우고, 실제 컨트롤러(action server)는
아무도 제공하지 않는다. 이 노드가 그 빈자리를 채운다:

  1. /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory)
       MoveIt 이 계획한 trajectory 를 받아 시간축을 따라 보간하여
       Isaac 명령 토픽으로 sensor_msgs/JointState 를 스트리밍.
  2. /gripper_controller/gripper_cmd (control_msgs/GripperCommand)
       그리퍼 목표 위치를 받아 같은 명령 토픽으로 반영.
  3. Isaac 상태 토픽 -> /joint_states 릴레이 (move_group 의 planning scene monitor 용).
     Isaac 이 이미 /joint_states 로 직접 내보내면 relay_joint_states:=false.

컨트롤러 이름/네임스페이스는 turtlebot3_manipulation_moveit_config/config/moveit_controllers.yaml
과 반드시 일치해야 한다 (arm_controller / follow_joint_trajectory,
gripper_controller / gripper_cmd).

실행:
  python3 moveit_to_isaac_bridge.py --ros-args \
      -p use_sim_time:=true \
      -p isaac_command_topic:=/isaac_joint_commands \
      -p isaac_state_topic:=/isaac_joint_states

그리퍼 확인 방법 (mimic 유지 여부):
  # Isaac 상태 토픽에 어떤 조인트가 나오는지
  ros2 topic echo <isaac_state_topic> --once
  #   -> name 에 gripper_right_joint 가 있으면 Isaac 이 독립 조인트로 취급
  #      => gripper_mirror:=equal (양쪽 다 명령)
  #   -> gripper_left_joint 만 있으면 오른쪽은 mimic/고정
  #      => gripper_mirror:=none (왼쪽만 명령)
  # 수동 명령으로 방향/범위 확인
  ros2 topic pub -r 20 <isaac_command_topic> sensor_msgs/msg/JointState \
      "{name: ['gripper_left_joint'], position: [0.01]}"    # open
  ros2 topic echo <isaac_state_topic>                       # right 가 따라오는지 관찰
"""

import bisect
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState


class MoveItToIsaacBridge(Node):
    def __init__(self):
        super().__init__('moveit_to_isaac_bridge')

        # ---- parameters -------------------------------------------------------
        self.isaac_command_topic = self.declare_parameter(
            'isaac_command_topic', '/isaac_joint_commands').value
        self.isaac_state_topic = self.declare_parameter(
            'isaac_state_topic', '/isaac_joint_states').value
        self.joint_states_out_topic = self.declare_parameter(
            'joint_states_out_topic', '/joint_states').value
        self.relay_joint_states = self.declare_parameter(
            'relay_joint_states', True).value

        self.arm_joints = self.declare_parameter(
            'arm_joints', ['joint1', 'joint2', 'joint3', 'joint4']).value
        self.gripper_left_joint = self.declare_parameter(
            'gripper_left_joint', 'gripper_left_joint').value
        self.gripper_right_joint = self.declare_parameter(
            'gripper_right_joint', 'gripper_right_joint').value
        # equal : right = left            (URDF mimic multiplier = 1)
        # negate: right = -left
        # none  : right 는 명령하지 않음   (Isaac 이 mimic 을 스스로 처리)
        self.gripper_mirror = self.declare_parameter('gripper_mirror', 'equal').value

        self.arm_action_name = self.declare_parameter(
            'arm_action_name', '/arm_controller/follow_joint_trajectory').value
        self.gripper_action_name = self.declare_parameter(
            'gripper_action_name', '/gripper_controller/gripper_cmd').value

        self.publish_rate = float(self.declare_parameter('publish_rate', 60.0).value)

        if self.relay_joint_states and self.isaac_state_topic == self.joint_states_out_topic:
            self.get_logger().warn(
                'isaac_state_topic == joint_states_out_topic; relay 를 비활성화한다.')
            self.relay_joint_states = False

        # ---- state ----------------------------------------------------------
        self._lock = threading.Lock()
        self._targets = {}          # joint name -> commanded position
        self._initialized = False   # Isaac 상태를 한 번이라도 받았는가

        cb = ReentrantCallbackGroup()

        # ---- I/O ----------------------------------------------------------
        self._cmd_pub = self.create_publisher(JointState, self.isaac_command_topic, 10)

        self._js_pub = None
        if self.relay_joint_states:
            self._js_pub = self.create_publisher(JointState, self.joint_states_out_topic, 10)

        self.create_subscription(
            JointState, self.isaac_state_topic, self._on_isaac_state, 10, callback_group=cb)

        self._timer = self.create_timer(
            1.0 / self.publish_rate, self._publish_command, callback_group=cb)

        self._arm_server = ActionServer(
            self, FollowJointTrajectory, self.arm_action_name,
            execute_callback=self._execute_arm,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self._gripper_server = ActionServer(
            self, GripperCommand, self.gripper_action_name,
            execute_callback=self._execute_gripper,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self.get_logger().info(
            f'bridge up | cmd->{self.isaac_command_topic} | state<-{self.isaac_state_topic} | '
            f'relay /joint_states={self.relay_joint_states} | gripper_mirror={self.gripper_mirror}')
        self.get_logger().info(
            f'arm action : {self.arm_action_name}  ({",".join(self.arm_joints)})')
        self.get_logger().info(
            f'gripper action : {self.gripper_action_name}  ({self.gripper_left_joint})')
        self.get_logger().info('Isaac 첫 상태 메시지 대기 중... (수신 전에는 명령을 보내지 않음)')

    # ------------------------------------------------------------------ state
    def _on_isaac_state(self, msg: JointState):
        if not self._initialized:
            managed = list(self.arm_joints) + [self.gripper_left_joint, self.gripper_right_joint]
            pos_by_name = dict(zip(msg.name, msg.position))
            with self._lock:
                for j in managed:
                    self._targets[j] = float(pos_by_name.get(j, 0.0))
                self._initialized = True
            self.get_logger().info(f'초기 타겟 확보: {self._targets}')

        if self._js_pub is not None:
            out = msg
            if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
                out.header.stamp = self.get_clock().now().to_msg()
            self._js_pub.publish(out)

    # ---------------------------------------------------------------- command
    def _publish_command(self):
        if not self._initialized:
            return
        with self._lock:
            names = list(self._targets.keys())
            positions = [self._targets[n] for n in names]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._cmd_pub.publish(msg)

    def _set_targets(self, mapping: dict):
        with self._lock:
            self._targets.update(mapping)

    # -------------------------------------------------------------- arm action
    def _execute_arm(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        points = list(traj.points)
        result = FollowJointTrajectory.Result()

        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
        total = times[-1]
        start = self.get_clock().now()
        rate = self.create_rate(self.publish_rate)

        self.get_logger().info(
            f'arm trajectory: {len(points)} pts, {total:.2f}s, joints={names}')

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('arm trajectory canceled')
                return FollowJointTrajectory.Result()

            now_s = (self.get_clock().now() - start).nanoseconds * 1e-9
            if now_s >= total:
                break

            i = min(max(bisect.bisect_right(times, now_s) - 1, 0), len(points) - 2)
            seg = times[i + 1] - times[i]
            alpha = 0.0 if seg <= 0.0 else (now_s - times[i]) / seg
            alpha = min(max(alpha, 0.0), 1.0)

            cmd = {}
            for k, name in enumerate(names):
                p0 = points[i].positions[k]
                p1 = points[i + 1].positions[k]
                cmd[name] = p0 + alpha * (p1 - p0)
            self._set_targets(cmd)

            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = names
            fb.desired.positions = [cmd[n] for n in names]
            goal_handle.publish_feedback(fb)

            rate.sleep()

        # 마지막 waypoint 로 정확히 고정
        self._set_targets({n: points[-1].positions[k] for k, n in enumerate(names)})

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('arm trajectory done')
        return result

    # ---------------------------------------------------------- gripper action
    def _execute_gripper(self, goal_handle):
        target = float(goal_handle.request.command.position)

        cmd = {self.gripper_left_joint: target}
        if self.gripper_mirror == 'equal':
            cmd[self.gripper_right_joint] = target
        elif self.gripper_mirror == 'negate':
            cmd[self.gripper_right_joint] = -target
        # 'none' -> 오른쪽은 건드리지 않음
        self._set_targets(cmd)
        self.get_logger().info(f'gripper -> {target:.4f}  ({cmd})')

        # 실제 위치 수렴을 별도로 확인하지 않고, 이동 시간만 기다린다.
        settle = self.create_rate(2.0)
        settle.sleep()

        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = target
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveItToIsaacBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
