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

#include "custom_nav2_bt_plugins/record_odom_pose_action.hpp"

namespace custom_nav2_bt_plugins
{

RecordOdomPoseAction::RecordOdomPoseAction(
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

BT::NodeStatus RecordOdomPoseAction::onStart()
{
  getInput("odom_topic", odom_topic_);
  getInput("timeout", timeout_);

  latest_odom_.reset();
  if (!odom_sub_ || odom_sub_->get_topic_name() != odom_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {latest_odom_ = msg;},
      opts);
  }

  start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RecordOdomPoseAction::onRunning()
{
  callback_group_executor_.spin_some();

  if (latest_odom_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = latest_odom_->header;
    pose.pose = latest_odom_->pose.pose;
    setOutput("pose", pose);
    RCLCPP_INFO(
      node_->get_logger(), "RecordOdomPose: recorded (%.3f, %.3f)",
      pose.pose.position.x, pose.pose.position.y);
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "RecordOdomPose: timed out waiting for /odom");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::RecordOdomPoseAction>("RecordOdomPose");
}
