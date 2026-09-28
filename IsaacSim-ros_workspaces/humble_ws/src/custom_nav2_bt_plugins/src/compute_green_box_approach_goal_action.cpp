// Copyright (c) 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <memory>
#include <string>

#include "custom_nav2_bt_plugins/compute_green_box_approach_goal_action.hpp"

#include "nav2_util/robot_utils.hpp"

namespace custom_nav2_bt_plugins
{

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

ComputeGreenBoxApproachGoalAction::ComputeGreenBoxApproachGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  std::string pose_topic = "green_box/pose";
  getInput("pose_topic", pose_topic);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ComputeGreenBoxApproachGoalAction::poseCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "ComputeGreenBoxApproachGoal: subscribed to \"%s\"", pose_topic.c_str());
}

void ComputeGreenBoxApproachGoalAction::poseCallback(
  geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_ = *msg;
  pose_received_ = true;
}

BT::NodeStatus ComputeGreenBoxApproachGoalAction::tick()
{
  callback_group_executor_.spin_some();

  double standoff = 0.5;
  double max_age = 1.0;
  double transform_tolerance = 0.1;
  std::string global_frame = "map";
  std::string robot_base_frame = "base_link";
  getInput("standoff_distance", standoff);
  getInput("max_age", max_age);
  getInput("transform_tolerance", transform_tolerance);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);

  geometry_msgs::msg::PoseStamped box;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_received_) {
      RCLCPP_WARN(
        node_->get_logger(), "ComputeGreenBoxApproachGoal: no green_box/pose received yet");
      return BT::NodeStatus::FAILURE;
    }
    // Freshness judged on the detector's capture stamp, matching the
    // contract table (pose timestamp decides staleness, not detected flag).
    const double age = (node_->now() - last_pose_.header.stamp).seconds();
    if (age < 0.0 || age > max_age) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ComputeGreenBoxApproachGoal: stale pose (age=%.2fs > max_age=%.2fs)", age, max_age);
      return BT::NodeStatus::FAILURE;
    }
    box = last_pose_;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_buffer_, global_frame, robot_base_frame, transform_tolerance))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "ComputeGreenBoxApproachGoal: TF lookup failed (\"%s\" <- \"%s\")",
      global_frame.c_str(), robot_base_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double bx = box.pose.position.x;
  const double by = box.pose.position.y;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;

  const double dx = bx - rx;
  const double dy = by - ry;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-3) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ComputeGreenBoxApproachGoal: robot is already at the box position, cannot derive a "
      "direction");
    return BT::NodeStatus::FAILURE;
  }
  const double ux = dx / dist;
  const double uy = dy / dist;
  const double yaw = std::atan2(dy, dx);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = global_frame;
  goal.header.stamp = node_->now();
  // Already inside the standoff: stay put (facing the box) instead of
  // sending the robot to a point behind it. The controller only drives
  // forward, so a goal behind the robot means turning round next to the box,
  // which stalled for minutes in g8 (the sim base turns poorly at low speed).
  // FinalApproachStop takes over from here. The goal keeps the robot's own
  // heading too: asking FollowPath to turn to the box would be the same slow
  // in-place turn; FinalApproachStop turns faster (max_angular_speed).
  const bool inside = dist <= standoff;
  goal.pose.position.x = inside ? rx : bx - standoff * ux;
  goal.pose.position.y = inside ? ry : by - standoff * uy;
  goal.pose.orientation = inside ? robot_pose.pose.orientation : yawToQuaternion(yaw);

  setOutput("approach_goal", goal);

  RCLCPP_INFO(
    node_->get_logger(),
    "ComputeGreenBoxApproachGoal: box (%.2f, %.2f) -> goal (%.2f, %.2f) yaw=%.2f in \"%s\"%s",
    bx, by, goal.pose.position.x, goal.pose.position.y, yaw, global_frame.c_str(),
    inside ? " (already within standoff - holding position)" : "");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::ComputeGreenBoxApproachGoalAction>(
    "ComputeGreenBoxApproachGoal");
}
