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

#ifndef CUSTOM_NAV2_BT_PLUGINS__APPROACH_OBSTACLE_NORMAL_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__APPROACH_OBSTACLE_NORMAL_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Build a short path that drives the robot straight at a movable
 * obstacle, stopping `standoff_distance` short of it and squared up to its face.
 *
 * Priority / gating:
 *  - projects the obstacle pose onto `path` (the latched INITIAL path from
 *    CaptureInitialPath);
 *  - FAILURE when the obstacle is farther than `on_path_tolerance` from that
 *    path - it is not the on-path blocker, so the normal planner handles it.
 *
 * Two geometry modes, chosen per tick:
 *  - surface-normal (preferred): when `use_surface_normal` is true AND the
 *    obstacle pose carries a usable, non-identity quaternion, that orientation
 *    is read as the surface normal yaw. The standoff goal is placed
 *    `standoff_distance` out along +normal (the free-space side) and every
 *    approach pose is oriented to face the obstacle (-normal) when
 *    `face_obstacle` is true.
 *  - path-tangent (legacy fallback): when there is no usable orientation, the
 *    initial-path tangent at the projection is used instead - goal =
 *    obstacle_xy - tangent * standoff_distance, heading = tangent yaw. This
 *    keeps behaviour identical to before while perception still publishes an
 *    identity orientation.
 *
 * approach path = straight, interpolated line from the robot's current pose to
 * that goal, every pose sharing the chosen heading.
 *
 * The result is latched and only recomputed when the obstacle moves more than
 * `recompute_threshold` (or the mode changes), so FollowPath is not fed a
 * jittering goal.
 */
class ApproachObstacleNormalAction : public BT::SyncActionNode
{
public:
  ApproachObstacleNormalAction(const std::string & name, const BT::NodeConfiguration & conf);
  ApproachObstacleNormalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Initial (pre-detour) global path"),
      BT::InputPort<std::string>(
        "pose_topic", std::string("/movable_obstacle/pose"),
        "geometry_msgs/PoseStamped topic with the obstacle position (map frame)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "obstacle_pose",
        "Frozen obstacle pose (e.g. from LockApproachPose). When set it is used "
        "instead of the pose_topic subscription and the staleness gate is skipped"),
      BT::InputPort<double>("standoff_distance", 0.4, "Stop this far [m] in front of the obstacle"),
      BT::InputPort<bool>(
        "use_surface_normal", true,
        "Use the obstacle pose orientation (surface normal) when it is a valid, "
        "non-identity quaternion; fall back to the initial-path tangent otherwise"),
      BT::InputPort<bool>(
        "face_obstacle", true,
        "Surface-normal mode: orient every approach pose to face the obstacle "
        "(along -normal). If false, face along +normal"),
      BT::InputPort<double>(
        "on_path_tolerance", 0.35,
        "Max lateral distance [m] of the obstacle from the path to count as 'on path'"),
      BT::InputPort<double>("pose_timeout", 2.0, "Max age [s] of the last obstacle pose"),
      BT::InputPort<double>("recompute_threshold", 0.2, "Rebuild if the obstacle moved this far [m]"),
      BT::InputPort<double>("interp_spacing", 0.1, "Spacing [m] of interpolated approach poses"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>("transform_tolerance", 0.1, "TF tolerance [s]"),
      BT::OutputPort<nav_msgs::msg::Path>("approach_path", "Straight approach path to follow"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("approach_goal", "Approach path end pose"),
    };
  }

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_pose_time_;
  bool pose_received_;

  std::string pose_topic_;
  double pose_timeout_;

  // Latched result.
  nav_msgs::msg::Path cached_approach_;
  geometry_msgs::msg::PoseStamped cached_goal_;
  double cached_obs_x_ {0.0};
  double cached_obs_y_ {0.0};
  bool cached_have_normal_ {false};
  bool have_cache_ {false};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__APPROACH_OBSTACLE_NORMAL_ACTION_HPP_
