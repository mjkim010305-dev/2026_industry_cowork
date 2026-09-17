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

#ifndef CUSTOM_NAV2_BT_PLUGINS__WIGGLE_IN_PLACE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__WIGGLE_IN_PLACE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Blind in-place rotation: sweep +angle, then -angle, then back to
 * the starting heading, to give AMCL fresh scan-matching bearings before
 * trusting its pose estimate.
 *
 * Why: after OdomLimitedDrive pushes against a resisting obstacle, wheel
 * slip can make /odom over-report actual translation. AMCL uses odom as
 * its motion-model control input between scan updates, and pure straight-
 * line driving (this project's push + drive-forward + drive-back-out) is a
 * weak-observability motion for MCL - scan matching along one axis often
 * can't fully correct a longitudinal bias picked up from a bad motion-
 * model prediction, especially in feature-sparse corridors. Left
 * unaddressed, AMCL's pose estimate keeps a residual forward bias, so the
 * return leg's goal checker (comparing AMCL's pose to the fixed
 * {spot_a_goal}) reports "arrived" while the robot is still physically
 * short of it.
 *
 * A deliberate rotation gives the particle filter genuinely different
 * bearings to scan-match against, which resolves this kind of bias far
 * better than more straight-line motion would - a well-known MCL
 * technique, and one that doesn't need to know or guess how much slip
 * actually happened (unlike hand-tuning a fixed distance offset, which
 * would need re-tuning any time the push distance/friction changes).
 *
 * Pure blind /cmd_vel angular control via odom yaw feedback (no map/
 * costmap involved, same family as OdomLimitedDrive/DriveToRecordedPose) -
 * not nav2's built-in Spin behavior, which collision-checks its sweep
 * against the local costmap and has reliably refused to operate this close
 * to an obstacle the robot itself just interacted with, same as
 * DriveOnHeading/BackUp did earlier in this same leg.
 *
 * Ends back at (approximately) the starting heading, so it doesn't disturb
 * the yaw=goal-orientation assumption ComputeStraightPathToPose/
 * FollowPathReverse depend on afterward.
 *
 * Non-blocking: StatefulActionNode, publishes /cmd_vel at ~20Hz.
 * FAILURE if `time_allowance` elapses before finishing the sweep.
 */
class WiggleInPlaceAction : public BT::StatefulActionNode
{
public:
  WiggleInPlaceAction(const std::string & name, const BT::NodeConfiguration & conf);
  WiggleInPlaceAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<std::string>("odom_topic", std::string("/odom"), "Odometry topic to track"),
      BT::InputPort<double>("angle", 0.3, "Sweep magnitude [rad] each side of the starting heading"),
      BT::InputPort<double>("angular_speed", 0.4, "Rotation speed [rad/s]"),
      BT::InputPort<double>("yaw_tolerance", 0.05, "Yaw tolerance [rad] per phase"),
      BT::InputPort<double>("time_allowance", 15.0, "Give up after this many seconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { WAIT_FOR_ODOM, SWEEP_POS, SWEEP_NEG, RETURN };

  void publishZero();
  bool rotateToward(double target_yaw);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double angle_ {0.3};
  double angular_speed_ {0.4};
  double yaw_tolerance_ {0.05};
  double time_allowance_ {15.0};

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Phase phase_ {Phase::WAIT_FOR_ODOM};
  double start_yaw_ {0.0};
  rclcpp::Time start_time_;
  rclcpp::Time last_log_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__WIGGLE_IN_PLACE_ACTION_HPP_
