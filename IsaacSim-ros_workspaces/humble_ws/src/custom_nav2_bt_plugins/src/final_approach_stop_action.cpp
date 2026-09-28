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

#include "custom_nav2_bt_plugins/final_approach_stop_action.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace custom_nav2_bt_plugins
{

FinalApproachStopAction::FinalApproachStopAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  latest_scan_time_(0, 0, RCL_ROS_TIME),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

void FinalApproachStopAction::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
  latest_scan_time_ = node_->now();
}

BT::NodeStatus FinalApproachStopAction::onStart()
{
  getInput("scan_topic", scan_topic_);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("stop_distance", stop_distance_);
  getInput("approach_speed", approach_speed_);
  getInput("front_half_angle", front_half_angle_);
  getInput("scan_loss_timeout", scan_loss_timeout_);
  getInput("time_allowance", time_allowance_);

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  }
  if (!scan_sub_ || scan_sub_->get_topic_name() != scan_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    latest_scan_.reset();
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FinalApproachStopAction::scanCallback, this, std::placeholders::_1),
      opts);
  }

  start_time_ = node_->now();
  RCLCPP_INFO(
    node_->get_logger(),
    "FinalApproachStop: creeping forward at %.3f m/s, stop at range <= %.2fm in \"%s\"",
    approach_speed_, stop_distance_, scan_topic_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FinalApproachStopAction::onRunning()
{
  callback_group_executor_.spin_some();

  if ((node_->now() - start_time_).seconds() > time_allowance_) {
    publishZero();
    RCLCPP_ERROR(
      node_->get_logger(),
      "FinalApproachStop: time_allowance exceeded before reaching stop_distance");
    return BT::NodeStatus::FAILURE;
  }

  // This creep is blind between scans, so a missing/stale scan is a
  // fail-safe: stop and FAIL rather than keep driving open-loop on data we
  // no longer trust. scan_loss_timeout defaults to 0.5s - a few LaserScan
  // periods at typical 10-40Hz rates, enough to ride out one dropped frame,
  // but far below time_allowance so a real sensor/link loss is caught in
  // well under a second instead of creeping blind for the full 30s budget.
  const double scan_age = latest_scan_ ?
    (node_->now() - latest_scan_time_).seconds() : std::numeric_limits<double>::infinity();
  if (!latest_scan_ || scan_age > scan_loss_timeout_) {
    publishZero();
    RCLCPP_ERROR(
      node_->get_logger(),
      "FinalApproachStop: no scan within scan_loss_timeout (%.2fs, age %.2fs) - "
      "stopping rather than creep blind",
      scan_loss_timeout_, scan_age);
    return BT::NodeStatus::FAILURE;
  }

  const auto & scan = *latest_scan_;
  double min_range = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::abs(angle) > front_half_angle_) {
      continue;
    }
    const double r = scan.ranges[i];
    if (!std::isfinite(r)) {
      // No return on this beam - not proof of clear space, just no
      // evidence either way, so it cannot count toward min_range.
      continue;
    }
    if (r > scan.range_max) {
      continue;
    }
    // A finite reading below range_min means "closer than the sensor can
    // measure" - clamp to 0 so it still drives the stop instead of being
    // discarded as invalid.
    min_range = std::min(min_range, r < scan.range_min ? 0.0 : r);
  }

  if (std::isfinite(min_range) && min_range <= stop_distance_) {
    publishZero();
    RCLCPP_INFO(
      node_->get_logger(), "FinalApproachStop: min forward range %.2fm <= %.2fm, stopped",
      min_range, stop_distance_);
    return BT::NodeStatus::SUCCESS;
  }

  publishForward(approach_speed_);
  return BT::NodeStatus::RUNNING;
}

void FinalApproachStopAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "FinalApproachStop: halted");
  publishZero();
}

void FinalApproachStopAction::publishZero()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }
}

void FinalApproachStopAction::publishForward(double speed)
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd_vel_pub_->publish(cmd);
  }
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::FinalApproachStopAction>("FinalApproachStop");
}
