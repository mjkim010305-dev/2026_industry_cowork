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

#include "festa_demo/is_green_box_detected_condition.hpp"

namespace festa_demo
{

IsGreenBoxDetectedCondition::IsGreenBoxDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  pose_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string pose_topic = "green_box/pose";
  std::string visible_topic = "green_box/detected";
  getInput("pose_topic", pose_topic);
  getInput("visible_topic", visible_topic);
  getInput("max_age", max_age_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsGreenBoxDetectedCondition::poseCallback, this, std::placeholders::_1),
    sub_option);
  visible_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    visible_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&IsGreenBoxDetectedCondition::visibleCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "IsGreenBoxDetected: pose \"%s\", visible \"%s\", max_age=%.2fs",
    pose_topic.c_str(), visible_topic.c_str(), max_age_);
}

void IsGreenBoxDetectedCondition::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Freshness is judged on the message header stamp (detector's capture
  // time), not arrival time - matches the contract table's "freshness is
  // pose timestamp" note.
  pose_time_ = msg->header.stamp;
  pose_received_ = true;
}

void IsGreenBoxDetectedCondition::visibleCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  visible_value_ = msg->data;
  visible_received_ = true;
}

BT::NodeStatus IsGreenBoxDetectedCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!pose_received_ || !visible_received_ || !visible_value_) {
    return BT::NodeStatus::FAILURE;
  }

  const double age = (node_->now() - pose_time_).seconds();
  if (age < 0.0 || age > max_age_) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace festa_demo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<festa_demo::IsGreenBoxDetectedCondition>(
    "IsGreenBoxDetected");
}
