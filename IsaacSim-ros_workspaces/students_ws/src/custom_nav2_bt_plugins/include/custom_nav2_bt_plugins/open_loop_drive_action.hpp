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

#ifndef CUSTOM_NAV2_BT_PLUGINS__OPEN_LOOP_DRIVE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__OPEN_LOOP_DRIVE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Drive straight forward (or backward) for a fixed time, publishing
 * /cmd_vel directly - NO collision checking, NO costmap lookahead.
 *
 * nav2_behaviors' DriveOnHeading/BackUp share one behavior_server-wide
 * "simulate_ahead_time" parameter (NOT namespaced per plugin instance,
 * unlike acceleration_limit etc.) that makes them refuse to drive into
 * anything their lookahead sees as a collision - exactly what we want when
 * intentionally pushing an obstacle aside, but there's no way to disable it
 * for just one plugin instance without weakening it everywhere else
 * (e.g. the RecoveryActions BackUp, which SHOULD stay collision-aware).
 *
 * This node sidesteps that shared setting entirely: open-loop, time-based
 * (distance / |speed| seconds), publishing a constant Twist directly.
 * Since it's genuinely blind, only use it where driving into something on
 * purpose is the intent (e.g. PushObstacle's drive-into-the-obstacle leg).
 *
 * Non-blocking: StatefulActionNode, publishes at ~20Hz while running.
 */
class OpenLoopDriveAction : public BT::StatefulActionNode
{
public:
  OpenLoopDriveAction(const std::string & name, const BT::NodeConfiguration & conf);
  OpenLoopDriveAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<double>("distance", 0.3, "Distance to travel [m] (sign of speed sets direction)"),
      BT::InputPort<double>("speed", 0.1, "Linear speed [m/s], positive = forward"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishZero();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::string cmd_vel_topic_;

  double speed_ {0.1};
  double duration_ {0.0};
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__OPEN_LOOP_DRIVE_ACTION_HPP_
