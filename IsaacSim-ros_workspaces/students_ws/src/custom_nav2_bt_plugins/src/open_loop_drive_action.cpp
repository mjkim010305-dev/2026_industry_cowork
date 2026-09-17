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

#include "custom_nav2_bt_plugins/open_loop_drive_action.hpp"

#include <algorithm>
#include <cmath>

namespace custom_nav2_bt_plugins
{

OpenLoopDriveAction::OpenLoopDriveAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus OpenLoopDriveAction::onStart()
{
  double distance = 0.3;
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("distance", distance);
  getInput("speed", speed_);

  if (std::abs(speed_) < 1e-6) {
    RCLCPP_ERROR(node_->get_logger(), "OpenLoopDrive: speed must be non-zero");
    return BT::NodeStatus::FAILURE;
  }

  duration_ = std::abs(distance) / std::abs(speed_);

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  }

  start_time_ = node_->now();
  RCLCPP_INFO(
    node_->get_logger(),
    "OpenLoopDrive: driving at %.3f m/s for %.2fs (%.2fm, no collision check) on \"%s\"",
    speed_, duration_, distance, cmd_vel_topic_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OpenLoopDriveAction::onRunning()
{
  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= duration_) {
    publishZero();
    RCLCPP_INFO(node_->get_logger(), "OpenLoopDrive: done");
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = speed_;
  cmd_vel_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void OpenLoopDriveAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "OpenLoopDrive: halted");
  publishZero();
}

void OpenLoopDriveAction::publishZero()
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
  factory.registerNodeType<custom_nav2_bt_plugins::OpenLoopDriveAction>("OpenLoopDrive");
}
