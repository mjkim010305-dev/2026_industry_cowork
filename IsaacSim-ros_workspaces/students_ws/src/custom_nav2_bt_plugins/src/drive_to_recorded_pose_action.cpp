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

#include "custom_nav2_bt_plugins/drive_to_recorded_pose_action.hpp"

#include <algorithm>
#include <cmath>

namespace custom_nav2_bt_plugins
{

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

DriveToRecordedPoseAction::DriveToRecordedPoseAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus DriveToRecordedPoseAction::onStart()
{
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("odom_topic", odom_topic_);
  if (!getInput("target_pose", target_pose_)) {
    RCLCPP_ERROR(node_->get_logger(), "DriveToRecordedPose: 'target_pose' is required");
    return BT::NodeStatus::FAILURE;
  }
  getInput("speed", speed_);
  getInput("angular_speed", angular_speed_);
  getInput("xy_tolerance", xy_tolerance_);
  getInput("yaw_tolerance", yaw_tolerance_);
  getInput("time_allowance", time_allowance_);

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  }
  if (!odom_sub_ || odom_sub_->get_topic_name() != odom_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    latest_odom_.reset();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {latest_odom_ = msg;},
      opts);
  }

  phase_ = Phase::DRIVE_BACK;
  start_time_ = node_->now();
  RCLCPP_INFO(
    node_->get_logger(),
    "DriveToRecordedPose: returning to (%.3f, %.3f) via odom feedback",
    target_pose_.pose.position.x, target_pose_.pose.position.y);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveToRecordedPoseAction::onRunning()
{
  callback_group_executor_.spin_some();

  if ((node_->now() - start_time_).seconds() > time_allowance_) {
    publishZero();
    RCLCPP_ERROR(node_->get_logger(), "DriveToRecordedPose: time_allowance exceeded");
    return BT::NodeStatus::FAILURE;
  }

  if (!latest_odom_) {
    return BT::NodeStatus::RUNNING;
  }

  const double dx = latest_odom_->pose.pose.position.x - target_pose_.pose.position.x;
  const double dy = latest_odom_->pose.pose.position.y - target_pose_.pose.position.y;
  const double dist = std::hypot(dx, dy);

  if (phase_ == Phase::DRIVE_BACK) {
    if (dist <= xy_tolerance_) {
      phase_ = Phase::CORRECT_YAW;
    } else {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -std::abs(speed_);
      cmd_vel_pub_->publish(cmd);
      return BT::NodeStatus::RUNNING;
    }
  }

  // Phase::CORRECT_YAW
  const double current_yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
  const double target_yaw = yawFromQuaternion(target_pose_.pose.orientation);
  const double yaw_error = normalizeAngle(target_yaw - current_yaw);

  if (std::abs(yaw_error) <= yaw_tolerance_) {
    publishZero();
    RCLCPP_INFO(
      node_->get_logger(), "DriveToRecordedPose: done (dist=%.3fm, yaw_err=%.3frad)",
      dist, yaw_error);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  const double k = 1.0;
  cmd.angular.z = std::clamp(k * yaw_error, -angular_speed_, angular_speed_);
  cmd_vel_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void DriveToRecordedPoseAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "DriveToRecordedPose: halted");
  publishZero();
}

void DriveToRecordedPoseAction::publishZero()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::DriveToRecordedPoseAction>("DriveToRecordedPose");
}
