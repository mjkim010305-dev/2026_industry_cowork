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

#ifndef CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Final leg of the green-box approach: drive straight ahead, open-loop,
 * until the LIDAR says the box is right in front, then stop. No pushing.
 *
 * This deliberately bypasses Nav2's planner/controller/costmap for the last
 * stretch, same reasoning as OdomLimitedDriveAction in this package: costmap
 * inflation around the box would make "stop just in front of it" an
 * unreachable goal for ComputePathToPose/FollowPath (the inflated cost keeps
 * the goal itself inside the "too close to an obstacle" region). By the time
 * this leaf runs, ComputeGreenBoxApproachGoal + FollowPath have already put
 * the robot roughly square-on at `standoff_distance`, so a short blind
 * forward creep with a LIDAR range gate is enough and safer than trying to
 * coerce the costmap.
 *
 * onRunning: take the freshest /scan, look at ranges whose angle falls inside
 * [-front_half_angle, +front_half_angle] (LaserScan frame, 0 = forward), take
 * the minimum valid range r. r <= stop_distance -> publish zero Twist and
 * SUCCEED. Otherwise publish a forward Twist and stay RUNNING. A stale/never
 * scan does not fail outright (LIDAR frames can be dropped intermittently) -
 * it simply keeps creeping forward, bounded by `time_allowance` like the
 * other open-loop actions in this package.
 * onHalted always publishes zero, so a preempted approach never leaves the
 * robot creeping.
 */
class FinalApproachStopAction : public BT::StatefulActionNode
{
public:
  FinalApproachStopAction(const std::string & name, const BT::NodeConfiguration & conf);
  FinalApproachStopAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("scan_topic", std::string("/scan"), "LaserScan to gate on"),
      BT::InputPort<std::string>(
        "cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<double>("stop_distance", 0.4, "Stop once min forward range <= this [m]"),
      BT::InputPort<double>("approach_speed", 0.15, "Forward creep speed [m/s]"),
      BT::InputPort<double>(
        "front_half_angle", 0.26, "Half-angle [rad] of the forward cone (0=full width)"),
      BT::InputPort<double>(
        "scan_timeout", 1.0, "Max age [s] of the last scan before it is ignored (not a failure)"),
      BT::InputPort<double>(
        "time_allowance", 30.0, "FAILURE if stop_distance is not reached within this many seconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishZero();
  void publishForward(double speed);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  double stop_distance_ {0.4};
  double approach_speed_ {0.15};
  double front_half_angle_ {0.26};
  double scan_timeout_ {1.0};
  double time_allowance_ {30.0};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_
