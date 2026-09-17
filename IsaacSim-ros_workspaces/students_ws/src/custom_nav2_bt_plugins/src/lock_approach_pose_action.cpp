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
#include <memory>
#include <string>

#include "custom_nav2_bt_plugins/lock_approach_pose_action.hpp"

namespace custom_nav2_bt_plugins
{

LockApproachPoseAction::LockApproachPoseAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  last_pose_time_(0, 0, RCL_ROS_TIME),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string pose_topic = "/movable_obstacle/pose";
  getInput("pose_topic", pose_topic);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&LockApproachPoseAction::poseCallback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(node_->get_logger(), "LockApproachPose: subscribed to \"%s\"", pose_topic.c_str());
}

void LockApproachPoseAction::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_pose_ = *msg;
  last_pose_time_ = node_->now();
  pose_received_ = true;
}

bool LockApproachPoseAction::hasRealOrientation(const geometry_msgs::msg::Quaternion & q)
{
  const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (n2 < 1e-6) {
    return false;
  }
  return !(std::abs(q.w) >= 0.99995 &&
         std::abs(q.x) + std::abs(q.y) + std::abs(q.z) < 1e-3);
}

BT::NodeStatus LockApproachPoseAction::onStart()
{
  getInput("pose_timeout", pose_timeout_);
  getInput("acquire_timeout", acquire_timeout_);
  getInput("require_orientation", require_orientation_);

  locked_ = false;
  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LockApproachPoseAction::onRunning()
{
  callback_group_executor_.spin_some();

  if (locked_) {
    setOutput("locked_pose", locked_pose_);
    return BT::NodeStatus::SUCCESS;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool fresh = pose_received_ &&
      (node_->now() - last_pose_time_).seconds() <= pose_timeout_;
    const bool oriented = !require_orientation_ ||
      hasRealOrientation(last_pose_.pose.orientation);
    if (fresh && oriented) {
      locked_pose_ = last_pose_;
      locked_ = true;
    }
  }

  if (locked_) {
    setOutput("locked_pose", locked_pose_);
    RCLCPP_INFO(
      node_->get_logger(),
      "LockApproachPose: locked obstacle pose (%.2f, %.2f) in \"%s\"",
      locked_pose_.pose.position.x, locked_pose_.pose.position.y,
      locked_pose_.header.frame_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > acquire_timeout_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "LockApproachPose: no acceptable pose within %.1f s", acquire_timeout_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void LockApproachPoseAction::onHalted()
{
  locked_ = false;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::LockApproachPoseAction>("LockApproachPose");
}
