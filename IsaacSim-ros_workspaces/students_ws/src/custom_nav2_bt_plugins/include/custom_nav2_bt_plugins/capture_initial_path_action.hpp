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

#ifndef CUSTOM_NAV2_BT_PLUGINS__CAPTURE_INITIAL_PATH_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__CAPTURE_INITIAL_PATH_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Latch the *first* global path produced for the current goal and
 * expose it on `initial_path`.
 *
 * The movable-obstacle logic must decide "is the obstacle on the path I was
 * originally going to take?". The live `{path}` is replanned at 1 Hz and, once
 * the obstacle enters the costmap, it detours around it - which would make the
 * obstacle look "off path". This node captures `{path}` once (re-capturing only
 * when the goal moves by more than `goal_change_threshold`) so that decision is
 * made against the pre-detour route.
 *
 * Always returns SUCCESS once it holds a path (so it can sit at the front of a
 * ReactiveSequence and latch from the very first tick).
 */
class CaptureInitialPathAction : public BT::SyncActionNode
{
public:
  CaptureInitialPathAction(const std::string & name, const BT::NodeConfiguration & conf);
  CaptureInitialPathAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Live global path ({path})"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Current navigation goal ({goal})"),
      BT::InputPort<double>(
        "goal_change_threshold", 0.5,
        "Re-capture when the goal moves more than this [m]"),
      BT::OutputPort<nav_msgs::msg::Path>("initial_path", "The latched pre-detour path"),
    };
  }

private:
  nav_msgs::msg::Path latched_;
  geometry_msgs::msg::PoseStamped latched_goal_;
  bool have_ {false};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__CAPTURE_INITIAL_PATH_ACTION_HPP_
