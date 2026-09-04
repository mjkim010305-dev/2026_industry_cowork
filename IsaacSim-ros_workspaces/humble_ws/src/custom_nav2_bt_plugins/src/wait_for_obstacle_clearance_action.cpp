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

#include <memory>
#include <string>

#include "custom_nav2_bt_plugins/wait_for_obstacle_clearance_action.hpp"

namespace custom_nav2_bt_plugins
{

WaitForObstacleClearanceAction::WaitForObstacleClearanceAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf),
  obstacle_present_(true),
  msg_received_(false),
  last_msg_time_(0, 0, RCL_ROS_TIME),
  timeout_(0.0),
  clear_on_stale_(true),
  message_timeout_(3.0),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  getInput("timeout", timeout_);
  getInput("clear_on_stale", clear_on_stale_);
  getInput("message_timeout", message_timeout_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  present_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&WaitForObstacleClearanceAction::presentCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "WaitForObstacleClearance: subscribed to \"%s\"", topic_.c_str());
}

void WaitForObstacleClearanceAction::presentCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_present_ = msg->data;
  msg_received_ = true;
  last_msg_time_ = node_->now();
}

BT::NodeStatus WaitForObstacleClearanceAction::onStart()
{
  start_time_ = node_->now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Require a fresh reading during this wait before declaring clearance.
    msg_received_ = false;
    obstacle_present_ = true;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "WaitForObstacleClearance: holding position, waiting for the obstacle to be handled");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForObstacleClearanceAction::onRunning()
{
  callback_group_executor_.spin_some();
  return evaluate();
}

BT::NodeStatus WaitForObstacleClearanceAction::evaluate()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg_received_) {
    if (!obstacle_present_) {
      RCLCPP_INFO(node_->get_logger(), "WaitForObstacleClearance: obstacle handled, resuming");
      return BT::NodeStatus::SUCCESS;
    }
    const bool stale = (node_->now() - last_msg_time_).seconds() > message_timeout_;
    if (stale && clear_on_stale_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "WaitForObstacleClearance: obstacle status went stale, assuming handled");
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (timeout_ > 0.0 && (node_->now() - start_time_).seconds() > timeout_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "WaitForObstacleClearance: gave up after %.1f s, resuming navigation", timeout_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForObstacleClearanceAction::onHalted()
{
  RCLCPP_DEBUG(node_->get_logger(), "WaitForObstacleClearance: halted");
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::WaitForObstacleClearanceAction>(
    "WaitForObstacleClearance");
}
