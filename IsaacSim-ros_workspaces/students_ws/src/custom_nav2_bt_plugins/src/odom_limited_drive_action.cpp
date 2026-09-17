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

#include "custom_nav2_bt_plugins/odom_limited_drive_action.hpp"

#include <cmath>

namespace custom_nav2_bt_plugins
{

OdomLimitedDriveAction::OdomLimitedDriveAction(
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

BT::NodeStatus OdomLimitedDriveAction::onStart()
{
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("odom_topic", odom_topic_);
  getInput("distance", distance_);
  getInput("speed", speed_);
  getInput("time_allowance", time_allowance_);

  if (std::abs(speed_) < 1e-6) {
    RCLCPP_ERROR(node_->get_logger(), "OdomLimitedDrive: speed must be non-zero");
    return BT::NodeStatus::FAILURE;
  }

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

  have_start_ = false;
  start_time_ = node_->now();
  RCLCPP_INFO(
    node_->get_logger(),
    "OdomLimitedDrive: driving at %.3f m/s until %.2fm actually covered (odom-tracked)",
    speed_, distance_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomLimitedDriveAction::onRunning()
{
  callback_group_executor_.spin_some();

  if ((node_->now() - start_time_).seconds() > time_allowance_) {
    publishZero();
    RCLCPP_ERROR(
      node_->get_logger(),
      "OdomLimitedDrive: time_allowance exceeded before %.2fm was covered", distance_);
    return BT::NodeStatus::FAILURE;
  }

  if (latest_odom_) {
    const double x = latest_odom_->pose.pose.position.x;
    const double y = latest_odom_->pose.pose.position.y;
    if (!have_start_) {
      start_x_ = x;
      start_y_ = y;
      have_start_ = true;
    } else {
      const double traveled = std::hypot(x - start_x_, y - start_y_);
      if (traveled >= distance_) {
        publishZero();
        RCLCPP_INFO(node_->get_logger(), "OdomLimitedDrive: covered %.3fm, done", traveled);
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = speed_;
  cmd_vel_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void OdomLimitedDriveAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "OdomLimitedDrive: halted");
  publishZero();
}

void OdomLimitedDriveAction::publishZero()
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
  factory.registerNodeType<custom_nav2_bt_plugins::OdomLimitedDriveAction>("OdomLimitedDrive");
}
