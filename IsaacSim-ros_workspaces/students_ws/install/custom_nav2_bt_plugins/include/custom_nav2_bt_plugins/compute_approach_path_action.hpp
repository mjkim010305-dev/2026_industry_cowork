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

#ifndef CUSTOM_NAV2_BT_PLUGINS__COMPUTE_APPROACH_PATH_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__COMPUTE_APPROACH_PATH_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Build the "approach sub-goal" path: the original global path,
 * truncated so it ends `standoff_distance` metres before a movable obstacle
 * that lies on that path.
 *
 * Geometry (chosen interpretation of "orthogonal approach"): the robot stays
 * on the global path and drives straight in, stopping short of and facing the
 * obstacle. This node does NOT leave the path.
 *
 *  - subscribes to the obstacle pose (map frame) published by
 *    `rgbd_obstacle_localizer`;
 *  - projects that point onto the incoming `path`;
 *  - FAILURE when the obstacle is farther than `on_path_tolerance` from the
 *    path (obstacles that are not on the path are lower priority and are left
 *    to the normal planner);
 *  - otherwise writes the truncated path to `approach_path` and its last pose
 *    to `approach_goal`, then returns SUCCESS.
 */
class ComputeApproachPathAction : public BT::SyncActionNode
{
public:
  ComputeApproachPathAction(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ComputeApproachPathAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Original global path to truncate"),
      BT::InputPort<std::string>(
        "pose_topic", std::string("/movable_obstacle/pose"),
        "geometry_msgs/PoseStamped topic with the obstacle position (map frame)"),
      BT::InputPort<double>(
        "standoff_distance", 0.5,
        "Distance [m] in front of the obstacle at which to stop"),
      BT::InputPort<double>(
        "on_path_tolerance", 0.35,
        "Max lateral distance [m] of the obstacle from the path to count as 'on path'"),
      BT::InputPort<double>(
        "pose_timeout", 2.0, "Max age [s] of the last obstacle pose message"),
      BT::OutputPort<nav_msgs::msg::Path>("approach_path", "Truncated path to follow"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "approach_goal", "Last pose of the truncated path"),
    };
  }

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_pose_time_;
  bool pose_received_;

  std::string pose_topic_;
  double pose_timeout_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__COMPUTE_APPROACH_PATH_ACTION_HPP_
