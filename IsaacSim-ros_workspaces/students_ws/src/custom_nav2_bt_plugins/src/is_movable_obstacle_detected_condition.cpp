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

#include "custom_nav2_bt_plugins/is_movable_obstacle_detected_condition.hpp"

namespace custom_nav2_bt_plugins
{

IsMovableObstacleDetectedCondition::IsMovableObstacleDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  detected_(false),
  received_(false),
  last_msg_time_(0, 0, RCL_ROS_TIME),
  message_timeout_(2.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("topic", topic_);
  getInput("message_timeout", message_timeout_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  obstacle_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsMovableObstacleDetectedCondition::obstacleCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "IsMovableObstacleDetected: subscribed to \"%s\"", topic_.c_str());
}

void IsMovableObstacleDetectedCondition::obstacleCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  detected_ = msg->data;
  received_ = true;
  last_msg_time_ = node_->now();
}

BT::NodeStatus IsMovableObstacleDetectedCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_) {
    return BT::NodeStatus::FAILURE;
  }
  if ((node_->now() - last_msg_time_).seconds() > message_timeout_) {
    // Monitor went silent - fail safe to normal navigation.
    return BT::NodeStatus::FAILURE;
  }
  return detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::IsMovableObstacleDetectedCondition>(
    "IsMovableObstacleDetected");
}
