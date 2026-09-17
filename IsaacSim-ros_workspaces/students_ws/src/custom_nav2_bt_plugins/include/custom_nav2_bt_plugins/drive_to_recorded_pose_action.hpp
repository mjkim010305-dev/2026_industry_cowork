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

#ifndef CUSTOM_NAV2_BT_PLUGINS__DRIVE_TO_RECORDED_POSE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__DRIVE_TO_RECORDED_POSE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Drive STRAIGHT BACKWARD (odom feedback, no map/costmap involved)
 * until within `xy_tolerance` of a previously recorded pose (e.g. from
 * RecordOdomPose), then rotate in place to correct yaw to match it.
 *
 * Companion to RecordOdomPose/OdomLimitedDrive: undoes a straight-line push
 * precisely, regardless of how far the push itself actually traveled -
 * unlike a fixed-distance blind reverse (OpenLoopDrive with negative
 * speed), which only returns to the right place if the forward push
 * covered exactly the assumed distance.
 *
 * Deliberately NOT ComputePathToPose+FollowPath: that path goes through
 * nav2's costmap/goal-checker machinery, which - same as DriveOnHeading/
 * BackUp's lookahead - reliably refuses to approach a point that now has
 * the just-placed object sitting on/near it. Pure odometry dead-reckoning
 * sidesteps that entirely for this short, known-straight-line return.
 *
 * Non-blocking: StatefulActionNode, publishes /cmd_vel at ~20Hz.
 * FAILURE if `time_allowance` elapses before converging.
 */
class DriveToRecordedPoseAction : public BT::StatefulActionNode
{
public:
  DriveToRecordedPoseAction(const std::string & name, const BT::NodeConfiguration & conf);
  DriveToRecordedPoseAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"), "Twist topic to publish"),
      BT::InputPort<std::string>("odom_topic", std::string("/odom"), "Odometry topic to track"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Pose to return to (from RecordOdomPose)"),
      BT::InputPort<double>("speed", 0.3, "Backward driving speed magnitude [m/s]"),
      BT::InputPort<double>("angular_speed", 0.4, "Max yaw-correction speed [rad/s]"),
      BT::InputPort<double>("xy_tolerance", 0.05, "Position tolerance [m] before yaw correction"),
      BT::InputPort<double>("yaw_tolerance", 0.05, "Yaw tolerance [rad] to finish"),
      BT::InputPort<double>("time_allowance", 30.0, "Give up after this many seconds"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { DRIVE_BACK, CORRECT_YAW };

  void publishZero();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string cmd_vel_topic_;
  std::string odom_topic_;
  geometry_msgs::msg::PoseStamped target_pose_;
  double speed_ {0.3};
  double angular_speed_ {0.4};
  double xy_tolerance_ {0.05};
  double yaw_tolerance_ {0.05};
  double time_allowance_ {30.0};

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  Phase phase_ {Phase::DRIVE_BACK};
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__DRIVE_TO_RECORDED_POSE_ACTION_HPP_
