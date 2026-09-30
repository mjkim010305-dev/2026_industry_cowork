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

#ifndef FESTA_DEMO__IS_GREEN_BOX_DETECTED_CONDITION_HPP_
#define FESTA_DEMO__IS_GREEN_BOX_DETECTED_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace festa_demo
{

/**
 * @brief Gate the green-box approach branch on the detector node (Lane P,
 * green_box_approach package) actually having a fresh, positive detection.
 *
 * SUCCESS iff the latest `visible_topic` (std_msgs/Bool) is true AND the
 * latest `pose_topic` (geometry_msgs/PoseStamped) is no older than `max_age`.
 * Both conditions are required: `visible_topic` alone can be stale (detector
 * crashed but last value was true), and a fresh pose alone does not mean this
 * cycle's blob+range resolution succeeded (see Session 11 contract table).
 */
class IsGreenBoxDetectedCondition : public BT::ConditionNode
{
public:
  IsGreenBoxDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGreenBoxDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "pose_topic", std::string("green_box/pose"),
        "geometry_msgs/PoseStamped published by the green-box detector"),
      BT::InputPort<std::string>(
        "visible_topic", std::string("green_box/detected"),
        "std_msgs/Bool published every cycle by the green-box detector"),
      BT::InputPort<double>(
        "max_age", 1.0,
        "Max age [s] of the last pose_topic message before it counts as stale"),
    };
  }

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void visibleCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visible_sub_;

  std::mutex mutex_;
  rclcpp::Time pose_time_;
  bool pose_received_ {false};
  bool visible_value_ {false};
  bool visible_received_ {false};

  double max_age_ {1.0};
};

}  // namespace festa_demo

#endif  // FESTA_DEMO__IS_GREEN_BOX_DETECTED_CONDITION_HPP_
