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

#include "custom_nav2_bt_plugins/record_map_pose_action.hpp"

namespace custom_nav2_bt_plugins
{

RecordMapPoseAction::RecordMapPoseAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus RecordMapPoseAction::onStart()
{
  getInput("pose_topic", pose_topic_);
  getInput("timeout", timeout_);

  latest_pose_.reset();
  if (!pose_sub_ || pose_sub_->get_topic_name() != pose_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {latest_pose_ = msg;},
      opts);
  }

  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecordMapPoseAction::onRunning()
{
  callback_group_executor_.spin_some();

  if (latest_pose_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = latest_pose_->header;
    pose.pose = latest_pose_->pose.pose;
    setOutput("pose", pose);
    RCLCPP_INFO(
      node_->get_logger(), "RecordMapPose: recorded (%.3f, %.3f) in \"%s\"",
      pose.pose.position.x, pose.pose.position.y, pose.header.frame_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "RecordMapPose: timed out waiting for %s", pose_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::RecordMapPoseAction>("RecordMapPose");
}
