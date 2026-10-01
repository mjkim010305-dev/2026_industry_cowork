#!/usr/bin/env python3
"""Start the AI Festa scenario: optional initial pose, then one NavigateToPose goal.

1. If set_initial_pose, publish /initialpose until AMCL answers on /amcl_pose.
2. Wait for /navigate_to_pose and send the goal. bt_navigator rejects goals
   until its lifecycle is active, so a rejected goal is retried.
3. If return_to_start and the goal succeeded, send a second goal back to the
   initial pose (the box is swept only if it still blocks the way back).
4. Log each result and exit (the rest of the launch keeps running).
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_srvs.srv import Trigger

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def quat_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SendGoal(Node):

    def __init__(self):
        super().__init__('festa_send_goal')
        p = {n: self.declare_parameter(n, d).value for n, d in (
            ('goal_x', 0.0), ('goal_y', 0.0), ('goal_yaw', 0.0),
            ('set_initial_pose', True), ('initial_x', 0.0), ('initial_y', 0.0), ('initial_yaw', 0.0),
            ('return_to_start', True), ('frame_id', 'map'), ('goal_retries', 30))}
        self.p = p
        self.amcl_seen = False
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl,
                                 QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=QoSReliabilityPolicy.RELIABLE))
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_amcl(self, _msg):
        self.amcl_seen = True

    def spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p['frame_id']
        msg.pose.pose.position.x = float(self.p['initial_x'])
        msg.pose.pose.position.y = float(self.p['initial_y'])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat_z(float(self.p['initial_yaw']))
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        for i in range(60):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            self.spin_for(2.0)
            if self.amcl_seen:
                self.get_logger().info(
                    f"initial pose ({self.p['initial_x']:.2f}, {self.p['initial_y']:.2f}, "
                    f"{self.p['initial_yaw']:.2f}) accepted by AMCL")
                return True
            self.get_logger().info(f'waiting for AMCL to take the initial pose ({i + 1})')
        self.get_logger().error('AMCL never published amcl_pose; sending the goal anyway')
        return False

    def wait_nav2_active(self, timeout_s=180.0):
        # festa_demo: bt_navigator accepts goals before the lifecycle manager has
        # activated the rest of Nav2 (on the robot's Pi the velocity smoother came
        # up 1 s after the first goal). Ask the manager itself.
        cli = self.create_client(Trigger, 'lifecycle_manager_navigation/is_active')
        end = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            if cli.wait_for_service(timeout_sec=1.0):
                fut = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.result() is not None and fut.result().success:
                    self.get_logger().info('Nav2 is active')
                    return True
            self.spin_for(1.0)
        self.get_logger().warn('Nav2 not reported active; sending the goal anyway')
        return False

    def clear_costmaps(self):
        # festa_demo: until AMCL has the initial pose it assumes (0, 0, 0), so on
        # the robot every scan in the first ~30 s was marked rotated and shifted
        # into both costmaps (2026-10-01 R5: the whole horizontal leg lethal, the
        # planner failed in 3 ms, which the tree read as a blocked corridor).
        for name in ('global_costmap/clear_entirely_global_costmap',
                     'local_costmap/clear_entirely_local_costmap'):
            cli = self.create_client(ClearEntireCostmap, name)
            if not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'{name} not available')
                continue
            fut = cli.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('costmaps cleared after the initial pose')
        self.spin_for(3.0)   # let a few scans refill them at the right pose

    def send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.p['frame_id']
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quat_z(float(yaw))
        self.get_logger().info('waiting for navigate_to_pose')
        self.nav.wait_for_server()
        for attempt in range(int(self.p['goal_retries'])):
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            fut = self.nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            handle = fut.result()
            if handle is not None and handle.accepted:
                self.get_logger().info(f'goal ({x:.2f}, {y:.2f}, {yaw:.2f}) accepted')
                res = handle.get_result_async()
                rclpy.spin_until_future_complete(self, res)
                status = res.result().status
                self.get_logger().info(f'Goal finished with status: {STATUS.get(status, status)}')
                return status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().warn(f'goal rejected (bt_navigator not active yet?), retry {attempt + 1}')
            self.spin_for(3.0)
        self.get_logger().error('goal was never accepted')
        return False


def main():
    rclpy.init()
    node = SendGoal()
    node.wait_nav2_active()
    if node.p['set_initial_pose']:
        node.set_initial_pose()
        node.clear_costmaps()
    p = node.p
    ok = node.send(p['goal_x'], p['goal_y'], p['goal_yaw'])
    if ok and p['return_to_start']:
        node.get_logger().info('returning to the start')
        # Arrive back facing the way it came (initial_yaw + pi), so there is no
        # 180 deg turn next to the start-area walls (D2 drove into the stub there).
        node.send(p['initial_x'], p['initial_y'], p['initial_yaw'] + math.pi)
    node.get_logger().info('send_goal done')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
