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

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#include "custom_nav2_bt_plugins/is_obstacle_tracking_condition.hpp"

namespace custom_nav2_bt_plugins
{

namespace
{
std::string toUpper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return s;
}
}  // namespace

IsObstacleTrackingCondition::IsObstacleTrackingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  state_time_(0, 0, RCL_ROS_TIME),
  fallback_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string state_topic = "/perception_state";
  std::string fallback_topic = "/obstacle/is_movable";
  tracking_value_ = "TRACKING";
  getInput("state_topic", state_topic);
  getInput("tracking_value", tracking_value_);
  getInput("state_timeout", state_timeout_);
  getInput("fallback_topic", fallback_topic);
  getInput("fallback_timeout", fallback_timeout_);
  tracking_value_ = toUpper(tracking_value_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  state_sub_ = node_->create_subscription<std_msgs::msg::String>(
    state_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsObstacleTrackingCondition::stateCallback, this, std::placeholders::_1),
    sub_option);
  fallback_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    fallback_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsObstacleTrackingCondition::fallbackCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "IsObstacleTracking: state \"%s\" (==\"%s\"), fallback \"%s\"",
    state_topic.c_str(), tracking_value_.c_str(), fallback_topic.c_str());
}

void IsObstacleTrackingCondition::stateCallback(std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_value_ = toUpper(msg->data);
  state_received_ = true;
  state_time_ = node_->now();
}

void IsObstacleTrackingCondition::fallbackCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fallback_value_ = msg->data;
  fallback_received_ = true;
  fallback_time_ = node_->now();
}

BT::NodeStatus IsObstacleTrackingCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::Time now = node_->now();

  // Primary: the stabilised perception state, when it is being published.
  if (state_received_ && (now - state_time_).seconds() <= state_timeout_) {
    return (state_value_ == tracking_value_) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  // Fallback: legacy per-frame boolean, until the Stabilizer node is up.
  if (fallback_received_ && (now - fallback_time_).seconds() <= fallback_timeout_) {
    return fallback_value_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;   // nothing fresh -> normal navigation
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::IsObstacleTrackingCondition>(
    "IsObstacleTracking");
}
