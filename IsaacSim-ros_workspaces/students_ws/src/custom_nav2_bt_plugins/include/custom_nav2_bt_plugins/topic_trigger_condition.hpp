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

#ifndef CUSTOM_NAV2_BT_PLUGINS__TOPIC_TRIGGER_CONDITION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__TOPIC_TRIGGER_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Generic "fire once on command" gate for an on-demand branch, driven
 * by any std_msgs/Bool topic (the `topic` port makes it reusable, not tied to
 * one specific request).
 *
 * SUCCESS exactly once per false->true transition of the topic; FAILURE
 * otherwise - including every tick after that one SUCCESS, for as long as the
 * topic stays true. This is what lets it gate a branch that must run once per
 * external request instead of looping forever while the requester's last
 * message is still "true" on the topic:
 *
 *   ros2 topic pub --once /place_request std_msgs/msg/Bool "{data: true}"
 *
 * fires the guarded branch exactly once. Because the QoS is volatile and a
 * one-shot publisher's process exits right after, no further message ever
 * arrives - this condition simply never fires again until something publishes
 * `false` and then `true` again (re-arming it).
 */
class TopicTriggerCondition : public BT::ConditionNode
{
public:
  TopicTriggerCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TopicTriggerCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", std::string("/place_request"),
        "std_msgs/Bool topic; a false->true transition fires this condition once"),
    };
  }

private:
  void callback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  std::mutex mutex_;
  bool received_ {false};
  bool data_ {false};
  bool prev_high_ {false};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__TOPIC_TRIGGER_CONDITION_HPP_
