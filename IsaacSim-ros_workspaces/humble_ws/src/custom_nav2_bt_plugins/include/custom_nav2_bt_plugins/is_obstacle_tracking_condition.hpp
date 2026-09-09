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

#ifndef CUSTOM_NAV2_BT_PLUGINS__IS_OBSTACLE_TRACKING_CONDITION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__IS_OBSTACLE_TRACKING_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Condition node that gates the movable-obstacle branch on a
 * hysteresis-stabilised perception state instead of a raw per-frame flag.
 *
 * Per bt_perception_stabilizer_design.md the actual hysteresis (IDLE /
 * CANDIDATE / TRACKING state machine, convergence test, extrapolation) lives in
 * a separate "Perception Stabilizer" node that publishes `/perception_state`
 * (std_msgs/String). This BT node is only the thin reader: SUCCESS while that
 * string equals `tracking_value` and is fresh.
 *
 * Until the Stabilizer exists (task 3) `/perception_state` is never published,
 * so this node transparently falls back to the legacy boolean
 * `/obstacle/is_movable`: SUCCESS while that is true and fresh. As soon as
 * `/perception_state` starts arriving it takes over and the fallback is
 * ignored - no XML change needed at that point.
 *
 * Fail-safe: neither source fresh -> FAILURE -> normal navigation.
 */
class IsObstacleTrackingCondition : public BT::ConditionNode
{
public:
  IsObstacleTrackingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsObstacleTrackingCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "state_topic", std::string("/perception_state"),
        "std_msgs/String from the Perception Stabilizer (IDLE/CANDIDATE/TRACKING)"),
      BT::InputPort<std::string>(
        "tracking_value", std::string("TRACKING"),
        "state_topic value (case-insensitive) that counts as 'tracking'"),
      BT::InputPort<double>(
        "state_timeout", 1.0,
        "Max age [s] of the last state_topic message before it is ignored"),
      BT::InputPort<std::string>(
        "fallback_topic", std::string("/obstacle/is_movable"),
        "std_msgs/Bool used only while state_topic has never been seen / is stale"),
      BT::InputPort<double>(
        "fallback_timeout", 3.0,
        "Max age [s] of the last fallback_topic message"),
    };
  }

private:
  void stateCallback(std_msgs::msg::String::SharedPtr msg);
  void fallbackCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fallback_sub_;

  std::mutex mutex_;
  std::string state_value_;
  bool state_received_ {false};
  rclcpp::Time state_time_;
  bool fallback_value_ {false};
  bool fallback_received_ {false};
  rclcpp::Time fallback_time_;

  std::string tracking_value_;
  double state_timeout_ {1.0};
  double fallback_timeout_ {3.0};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__IS_OBSTACLE_TRACKING_CONDITION_HPP_
