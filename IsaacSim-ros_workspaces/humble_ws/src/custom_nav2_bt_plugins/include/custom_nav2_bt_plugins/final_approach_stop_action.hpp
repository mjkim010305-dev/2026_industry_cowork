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
 * the minimum range r among readings that carry any distance evidence:
 * finite r < range_min is closer than the sensor can report and is clamped
 * to 0 (it still drives the stop, it is not discarded); inf/NaN readings are
 * "no return" and are skipped (absence of a return is not proof the box is
 * gone, so it must not count as clear space either). min_range <=
 * stop_distance -> publish zero Twist and SUCCEED. Otherwise publish a
 * forward Twist and stay RUNNING.
 * This creep is open-loop and blind between scans, so a missing/stale scan
 * is a fail-safe, not a shrug: if no scan has ever arrived, or the latest
 * one is older than `scan_loss_timeout`, publish zero Twist and FAIL
 * immediately rather than let the robot keep creeping on stale data. One
 * transiently late scan inside that window is tolerated (falls through to
 * the same command as last tick); only a loss that outlasts the window
 * fails. `time_allowance` remains the separate, much larger bound on total
 * leaf runtime when scans are healthy but the box is never reached.
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
        "scan_loss_timeout", 0.5,
        "FAILURE if no scan has ever arrived or the latest one is older than this [s] "
        "(must stay well below time_allowance - this is the blind-creep fail-safe, "
        "not the overall runtime budget)"),
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
  double scan_loss_timeout_ {0.5};
  double time_allowance_ {30.0};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__FINAL_APPROACH_STOP_ACTION_HPP_
