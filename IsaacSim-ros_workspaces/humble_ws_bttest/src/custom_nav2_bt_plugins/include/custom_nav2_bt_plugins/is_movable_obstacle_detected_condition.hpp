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

#ifndef CUSTOM_NAV2_BT_PLUGINS__IS_MOVABLE_OBSTACLE_DETECTED_CONDITION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__IS_MOVABLE_OBSTACLE_DETECTED_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief A BT::ConditionNode that subscribes to a single std_msgs/Bool topic
 * and returns SUCCESS while that topic reports a movable obstacle.
 *
 * All perception (RGB-D depth processing, classification, proximity gating)
 * happens upstream in the `rgbd_obstacle_monitor` node / MovableObstacleDetector
 * plugin. This condition only reads the resulting boolean:
 *
 *  - fresh message with data == true  -> SUCCESS
 *  - data == false, or no message within `message_timeout` seconds -> FAILURE
 */
class IsMovableObstacleDetectedCondition : public BT::ConditionNode
{
public:
  IsMovableObstacleDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsMovableObstacleDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", std::string("/obstacle/is_movable"),
        "std_msgs/Bool topic from the VLM: true when the blocker is a movable obstacle"),
      BT::InputPort<double>(
        "message_timeout", 2.0,
        "Max age [s] of the last message; older is treated as 'no obstacle'"),
    };
  }

private:
  void obstacleCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;

  std::mutex mutex_;
  bool detected_;
  bool received_;
  rclcpp::Time last_msg_time_;

  std::string topic_;
  double message_timeout_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__IS_MOVABLE_OBSTACLE_DETECTED_CONDITION_HPP_
