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

#ifndef CUSTOM_NAV2_BT_PLUGINS__COMPUTE_STRAIGHT_PATH_TO_POSE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__COMPUTE_STRAIGHT_PATH_TO_POSE_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Build a STRAIGHT LINE path (in `global_frame`) from the robot's
 * current TF pose directly to `goal`, skipping the global planner
 * (ComputePathToPose) entirely.
 *
 * Why: ComputePathToPose queries the global costmap and refuses (or plans
 * around) a goal cell that reads as lethal - which is exactly what happens
 * when the goal is a point where the robot itself just placed an object
 * (e.g. returning to spot A after PushObstacle/OdomLimitedDrive). This
 * node sidesteps that query altogether, the same way OrientPathToYaw
 * sidesteps NavFn's discretization for the final approach: it just does
 * the geometry directly.
 *
 * Every waypoint (not just the tail, since the whole path IS the final
 * approach here) faces `goal`'s orientation - mirroring OrientPathToYaw's
 * approach of forcing a fixed heading across the final straight stretch so
 * PathAlign/GoalAlign converge heading during normal driving instead of
 * the slow terminal RotateToGoal-only phase.
 *
 * NOTE: this only removes the GLOBAL PLANNER's refusal to path near a
 * known obstacle. The LOCAL controller (FollowPath/DWB) still runs its own
 * costmap-based footprint collision check against the resulting path, and
 * may still refuse to approach if that isn't also addressed (e.g. by
 * disabling the local costmap's obstacle layer for the duration of this
 * return leg) - being tested first without that to see how far this gets
 * on its own.
 *
 * Fails if TF (`global_frame` -> `robot_base_frame`) isn't available, or
 * `goal` isn't provided.
 */
class ComputeStraightPathToPoseAction : public BT::SyncActionNode
{
public:
  ComputeStraightPathToPoseAction(const std::string & name, const BT::NodeConfiguration & conf);
  ComputeStraightPathToPoseAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target pose (in global_frame)"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Frame the path is built in"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot frame to look up the current pose"),
      BT::InputPort<double>(
        "step_distance", 0.1, "Spacing [m] between generated waypoints"),
      BT::OutputPort<nav_msgs::msg::Path>("path", "The generated straight-line path"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__COMPUTE_STRAIGHT_PATH_TO_POSE_ACTION_HPP_
