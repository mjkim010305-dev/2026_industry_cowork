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

#ifndef CUSTOM_NAV2_BT_PLUGINS__ODOM_LIMITED_DRIVE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__ODOM_LIMITED_DRIVE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Drive straight forward, no collision checking, but stop based on
 * ACTUAL distance traveled (measured via /odom deltas) instead of a fixed
 * time - open-loop time-based driving (OpenLoopDrive) could command
 * "8m at 1m/s = 8s" but if the robot stalls/slips pushing against an
 * obstacle's resistance, the real distance covered can be much less.
 *
 * Still no map/costmap/collision-check involved (same reasoning as
 * OpenLoopDrive - this is used to intentionally push into an obstacle),
 * just closes the loop on actual displacement via odometry instead of
 * blind timing.
 *
 * Non-blocking: StatefulActionNode, publishes /cmd_vel at ~20Hz.
 * FAILURE if `time_allowance` elapses before `distance` is actually
 * covered (e.g. the obstacle never budges at all).
 */
class OdomLimitedDriveAction : public BT::StatefulActionNode
{
public:
  OdomLimitedDriveAction(const std::string & name, const BT::NodeConfiguration & conf);
  OdomLimitedDriveAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<std::string>("odom_topic", std::string("/odom"), "Odometry topic to track"),
      BT::InputPort<double>("distance", 0.3, "Distance to actually cover [m]"),
      BT::InputPort<double>("speed", 0.1, "Linear speed [m/s], positive = forward"),
      BT::InputPort<double>("time_allowance", 30.0, "Give up after this many seconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishZero();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double distance_ {0.3};
  double speed_ {0.1};
  double time_allowance_ {30.0};

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  bool have_start_ {false};
  double start_x_ {0.0};
  double start_y_ {0.0};
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__ODOM_LIMITED_DRIVE_ACTION_HPP_
