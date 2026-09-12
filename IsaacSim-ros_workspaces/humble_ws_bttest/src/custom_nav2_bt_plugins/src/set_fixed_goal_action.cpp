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

#include "custom_nav2_bt_plugins/set_fixed_goal_action.hpp"

#include <cmath>
#include <string>

namespace custom_nav2_bt_plugins
{

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

SetFixedGoalAction::SetFixedGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus SetFixedGoalAction::tick()
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string frame = "map";

  if (!getInput("x", x) || !getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "SetFixedGoal: 'x' and 'y' are required");
    return BT::NodeStatus::FAILURE;
  }
  getInput("yaw", yaw);
  getInput("frame", frame);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame;
  goal.header.stamp = node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = yawToQuaternion(yaw);
  setOutput("goal", goal);

  RCLCPP_INFO(
    node_->get_logger(),
    "SetFixedGoal: goal set to (%.2f, %.2f) yaw=%.2f in \"%s\"",
    x, y, yaw, frame.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::SetFixedGoalAction>("SetFixedGoal");
}
