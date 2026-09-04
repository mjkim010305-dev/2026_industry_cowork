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

#include <cmath>
#include <string>

#include "custom_nav2_bt_plugins/capture_initial_path_action.hpp"

namespace custom_nav2_bt_plugins
{

CaptureInitialPathAction::CaptureInitialPathAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus CaptureInitialPathAction::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  geometry_msgs::msg::PoseStamped goal;
  const bool have_goal = static_cast<bool>(getInput("goal", goal));

  double thr = 0.5;
  getInput("goal_change_threshold", thr);

  bool relatch = !have_;
  if (have_ && have_goal) {
    const double d = std::hypot(
      goal.pose.position.x - latched_goal_.pose.position.x,
      goal.pose.position.y - latched_goal_.pose.position.y);
    if (d > thr) {
      relatch = true;
    }
  }

  if (relatch && !path.poses.empty()) {
    latched_ = path;
    if (have_goal) {
      latched_goal_ = goal;
    }
    have_ = true;
  }

  if (!have_) {
    return BT::NodeStatus::FAILURE;   // no path captured yet
  }

  setOutput("initial_path", latched_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::CaptureInitialPathAction>("CaptureInitialPath");
}
