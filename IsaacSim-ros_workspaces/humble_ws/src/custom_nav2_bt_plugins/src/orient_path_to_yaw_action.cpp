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

#include "custom_nav2_bt_plugins/orient_path_to_yaw_action.hpp"

#include <cmath>

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

double poseDistance(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}
}  // namespace

OrientPathToYawAction::OrientPathToYawAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus OrientPathToYawAction::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  if (path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "OrientPathToYaw: input path is empty");
    return BT::NodeStatus::FAILURE;
  }

  double yaw = 0.0;
  double align_distance = 2.0;
  double step_distance = 0.1;
  getInput("yaw", yaw);
  getInput("align_distance", align_distance);
  getInput("step_distance", step_distance);
  if (step_distance <= 0.0) {
    step_distance = 0.1;
  }

  const auto q = yawToQuaternion(yaw);
  const geometry_msgs::msg::PoseStamped goal_pose = path.poses.back();

  // Walk backward from the last pose, accumulating segment lengths, to find
  // how many trailing poses fall within align_distance of the goal - those
  // get dropped and replaced by the freshly built straight segment below.
  std::size_t keep_count = path.poses.size() - 1;  // poses to keep, before the tail
  double accumulated = 0.0;
  for (std::size_t i = path.poses.size() - 1; i-- > 0; ) {
    accumulated += poseDistance(path.poses[i], path.poses[i + 1]);
    if (accumulated > align_distance) {
      keep_count = i + 1;
      break;
    }
    keep_count = i;
  }
  path.poses.resize(keep_count);

  // Build a straight line ending exactly at goal_pose's position, all
  // points facing `yaw`, starting align_distance back along -yaw.
  const double dx = std::cos(yaw);
  const double dy = std::sin(yaw);
  const double start_x = goal_pose.pose.position.x - align_distance * dx;
  const double start_y = goal_pose.pose.position.y - align_distance * dy;

  const int n_steps = std::max(1, static_cast<int>(std::round(align_distance / step_distance)));
  for (int i = 0; i <= n_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_steps);
    geometry_msgs::msg::PoseStamped p = goal_pose;
    p.pose.position.x = start_x + t * (goal_pose.pose.position.x - start_x);
    p.pose.position.y = start_y + t * (goal_pose.pose.position.y - start_y);
    p.pose.orientation = q;
    path.poses.push_back(p);
  }

  setOutput("oriented_path", path);
  RCLCPP_INFO(
    node_->get_logger(),
    "OrientPathToYaw: replaced the last %.2fm with a straight %d-point segment at yaw=%.2f "
    "(%zu poses total)",
    align_distance, n_steps + 1, yaw, path.poses.size());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::OrientPathToYawAction>("OrientPathToYaw");
}
