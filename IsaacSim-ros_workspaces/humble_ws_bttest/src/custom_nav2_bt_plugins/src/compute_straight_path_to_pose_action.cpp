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

#include "custom_nav2_bt_plugins/compute_straight_path_to_pose_action.hpp"

#include <cmath>
#include <algorithm>

#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace custom_nav2_bt_plugins
{

ComputeStraightPathToPoseAction::ComputeStraightPathToPoseAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus ComputeStraightPathToPoseAction::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "ComputeStraightPathToPose: 'goal' is required");
    return BT::NodeStatus::FAILURE;
  }

  std::string global_frame = "map";
  std::string robot_base_frame = "base_link";
  double step_distance = 0.1;
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("step_distance", step_distance);
  if (step_distance <= 0.0) {
    step_distance = 0.1;
  }

  geometry_msgs::msg::TransformStamped current_tf;
  try {
    current_tf = tf_buffer_->lookupTransform(
      global_frame, robot_base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "ComputeStraightPathToPose: TF %s -> %s unavailable: %s",
      global_frame.c_str(), robot_base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double start_x = current_tf.transform.translation.x;
  const double start_y = current_tf.transform.translation.y;
  const double dx = goal.pose.position.x - start_x;
  const double dy = goal.pose.position.y - start_y;
  const double distance = std::hypot(dx, dy);

  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame;
  path.header.stamp = node_->now();

  const int n_steps = std::max(1, static_cast<int>(std::round(distance / step_distance)));
  for (int i = 0; i <= n_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_steps);
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = start_x + t * dx;
    p.pose.position.y = start_y + t * dy;
    p.pose.orientation = goal.pose.orientation;
    path.poses.push_back(p);
  }

  setOutput("path", path);
  RCLCPP_INFO(
    node_->get_logger(),
    "ComputeStraightPathToPose: built a %.2fm straight path (%d points) from "
    "(%.3f, %.3f) to (%.3f, %.3f) in \"%s\"",
    distance, n_steps + 1, start_x, start_y,
    goal.pose.position.x, goal.pose.position.y, global_frame.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::ComputeStraightPathToPoseAction>(
    "ComputeStraightPathToPose");
}
