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

#ifndef CUSTOM_NAV2_BT_PLUGINS__SET_FIXED_GOAL_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__SET_FIXED_GOAL_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Write a hardcoded map-frame coordinate (given in the XML) to a
 * PoseStamped output port, so the robot can be driven to a fixed location
 * without an external NavigateToPose goal supplying it.
 *
 * This node does not plan or drive by itself - pair it with the existing
 * nav2 ComputePathToPose / FollowPath BT nodes (already in plugin_lib_names)
 * to actually move there:
 *
 *   <SetFixedGoal x="2.0" y="1.0" yaw="0.0" goal="{place_goal}"/>
 *   <ComputePathToPose goal="{place_goal}" path="{place_path}" planner_id="GridBased"/>
 *   <FollowPath path="{place_path}" controller_id="FollowPath"/>
 *
 * Use a blackboard key other than "goal"/"path" (e.g. "place_goal") when this
 * runs inside a tree that already uses "{goal}"/"{path}" for the outer
 * NavigateToPose request, to avoid overwriting it.
 *
 * Always SUCCESS unless x/y are missing - pure port assignment, no I/O.
 */
class SetFixedGoalAction : public BT::SyncActionNode
{
public:
  SetFixedGoalAction(const std::string & name, const BT::NodeConfiguration & conf);
  SetFixedGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "Target X [m] in `frame`"),
      BT::InputPort<double>("y", "Target Y [m] in `frame`"),
      BT::InputPort<double>("yaw", 0.0, "Target heading [rad]"),
      BT::InputPort<std::string>(
        "frame", std::string("map"), "Frame the coordinate is expressed in"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "The resulting PoseStamped"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__SET_FIXED_GOAL_ACTION_HPP_
