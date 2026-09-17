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

#ifndef CUSTOM_NAV2_BT_PLUGINS__ORIENT_PATH_TO_YAW_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__ORIENT_PATH_TO_YAW_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Replace the last `align_distance` meters of a path with a freshly
 * built STRAIGHT LINE of new waypoints, all facing a fixed yaw, ending
 * exactly at the path's original goal position.
 *
 * Why: DWB's RotateToGoal critic only kicks in once the robot is already
 * within xy_goal_tolerance of the goal, and it's a slow, special-cased
 * "stop translating, rotate in place" behavior - observed to take 100+
 * seconds to close the last few degrees, which SimpleProgressChecker (it
 * only tracks translation) eventually times out on even though the
 * rotation is genuinely converging.
 *
 * Instead: build the final stretch of the path itself as a clean straight
 * line at the desired final yaw - not just relabeling the orientation of
 * whatever poses NavFn happened to put there (their positions can still
 * wobble slightly from grid discretization, which would fight the
 * orientation we're forcing). The new segment runs from
 * (goal_position - align_distance * [cos(yaw), sin(yaw)]) to goal_position,
 * sampled every `step_distance` meters, every point facing `yaw`.
 * PathAlign/GoalAlign (already weighted highly in this project's DWB
 * config) then pull the robot to match that heading as part of NORMAL
 * forward-driving trajectory scoring for the whole final stretch, not the
 * special terminal rotate-only mode - so by the time the robot crosses
 * into xy_goal_tolerance it's already facing the right way and the
 * residual RotateToGoal correction (if any) is small and finishes quickly.
 *
 * Only the tail of the path is replaced (poses within align_distance of
 * the last pose, walking backward accumulating segment lengths, are
 * dropped and replaced by the new straight segment) so the earlier,
 * possibly curvy/obstacle-avoiding part of the route is left as planned -
 * forcing the WHOLE path to face one fixed yaw would fight the planner on
 * turns before the robot is anywhere near the goal.
 *
 * Always returns SUCCESS if given a non-empty path (fails on empty path).
 */
class OrientPathToYawAction : public BT::SyncActionNode
{
public:
  OrientPathToYawAction(const std::string & name, const BT::NodeConfiguration & conf);
  OrientPathToYawAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Input path whose tail gets replaced"),
      BT::InputPort<double>("yaw", 0.0, "Target yaw [rad] to face at the end of the path"),
      BT::InputPort<double>(
        "align_distance", 2.0,
        "How many meters back from the goal to replace with a straight segment"),
      BT::InputPort<double>(
        "step_distance", 0.1,
        "Spacing [m] between generated waypoints in the straight segment"),
      BT::OutputPort<nav_msgs::msg::Path>("oriented_path", "The re-built path"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__ORIENT_PATH_TO_YAW_ACTION_HPP_
