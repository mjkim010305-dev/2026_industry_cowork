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

#ifndef CUSTOM_NAV2_BT_PLUGINS__RECORD_ODOM_POSE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__RECORD_ODOM_POSE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Snapshot the robot's current /odom pose onto the blackboard.
 *
 * Purely odometry-based (no map/costmap involved at all), so it's immune
 * to the "goal coincides with a just-placed object" collision-check
 * problem that map-frame navigation (ComputePathToPose/FollowPath, or even
 * DriveOnHeading/BackUp's lookahead) kept hitting near spot A. Meant to be
 * called right before an OdomLimitedDrive push, so DriveToRecordedPose can
 * later return to exactly this pose regardless of how far the push
 * actually traveled.
 *
 * StatefulActionNode: RUNNING until at least one /odom message has been
 * received (bounded by timeout), then SUCCESS.
 */
class RecordOdomPoseAction : public BT::StatefulActionNode
{
public:
  RecordOdomPoseAction(const std::string & name, const BT::NodeConfiguration & conf);
  RecordOdomPoseAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("odom_topic", std::string("/odom"), "Odometry topic"),
      BT::InputPort<double>("timeout", 3.0, "Seconds to wait for an /odom message"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "The recorded pose"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::string odom_topic_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  rclcpp::Time start_time_;
  double timeout_ {3.0};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__RECORD_ODOM_POSE_ACTION_HPP_
