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

#include "custom_nav2_bt_plugins/topic_trigger_condition.hpp"

#include <memory>
#include <string>

namespace custom_nav2_bt_plugins
{

TopicTriggerCondition::TopicTriggerCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic = "/place_request";
  getInput("topic", topic);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&TopicTriggerCondition::callback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(),
    "TopicTrigger: watching \"%s\" for a rising edge", topic.c_str());
}

void TopicTriggerCondition::callback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_ = msg->data;
  received_ = true;
}

BT::NodeStatus TopicTriggerCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_) {
    return BT::NodeStatus::FAILURE;
  }

  const bool fire = data_ && !prev_high_;
  prev_high_ = data_;

  if (fire) {
    RCLCPP_INFO(node_->get_logger(), "TopicTrigger: fired");
  }
  return fire ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::TopicTriggerCondition>("TopicTrigger");
}
