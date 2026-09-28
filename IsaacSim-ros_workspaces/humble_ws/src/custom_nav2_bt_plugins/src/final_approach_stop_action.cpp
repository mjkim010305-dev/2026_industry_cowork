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

#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace custom_nav2_bt_plugins
{

namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}
}  // namespace

FinalApproachStopAction::FinalApproachStopAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  latest_scan_time_(0, 0, RCL_ROS_TIME),
  start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
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

void FinalApproachStopAction::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  // g8: at contact range the detector's pose goes unreliable (its close-range
  // geometry breaks down), which kept moving the steering target. The box is
  // static, so only the first pose after onStart is kept for the rest of this
  // run - later messages are ignored until the next onStart resets the flag.
  if (pose_received_since_start_) {
    return;
  }
  last_box_pose_ = *msg;
  pose_received_since_start_ = true;
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
  getInput("pose_topic", pose_topic_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("align_threshold", align_threshold_);
  getInput("max_angular_speed", max_angular_speed_);
  getInput("heading_gain", heading_gain_);
  getInput("stop_center_distance", stop_center_distance_);

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
  if (!pose_sub_ || pose_sub_->get_topic_name() != pose_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&FinalApproachStopAction::poseCallback, this, std::placeholders::_1),
      opts);
  }
  // Reset every onStart (not only on resubscribe, unlike latest_scan_ above):
  // g6 crept on whatever heading the robot already had, so a fresh run of
  // this leaf must wait for a fresh bearing fix rather than trust a pose
  // received during some earlier run of the leaf.
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_received_since_start_ = false;
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
  // Measured from onStart when nothing has arrived yet: the first scan after
  // the node starts is "not yet", not "lost" (g3 failed twice on the first
  // tick with age inf). Stand still while waiting - never creep before the
  // first scan.
  const double scan_age = latest_scan_ ?
    (node_->now() - latest_scan_time_).seconds() : (node_->now() - start_time_).seconds();
  if (!latest_scan_ && scan_age <= scan_loss_timeout_) {
    publishZero();
    return BT::NodeStatus::RUNNING;
  }
  if (!latest_scan_ || scan_age > scan_loss_timeout_) {
    publishZero();
    RCLCPP_ERROR(
      node_->get_logger(),
      "FinalApproachStop: no scan within scan_loss_timeout (%.2fs, age %.2fs) - "
      "stopping rather than creep blind",
      scan_loss_timeout_, scan_age);
    return BT::NodeStatus::FAILURE;
  }

  // g6: this leaf must not creep on whatever heading the robot happens to
  // have, so it needs a box bearing before it may move at all. The box is
  // static, so once a pose_topic message has arrived it stays valid for the
  // rest of this run (no per-tick freshness check, unlike the scan above) -
  // but the same grace-then-fail shape as the first-scan handling applies
  // while waiting for that first message, reusing scan_loss_timeout as the
  // grace window (see header).
  geometry_msgs::msg::PoseStamped box_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_received_since_start_) {
      const double pose_age = (node_->now() - start_time_).seconds();
      publishZero();
      if (pose_age <= scan_loss_timeout_) {
        return BT::NodeStatus::RUNNING;
      }
      RCLCPP_ERROR(
        node_->get_logger(),
        "FinalApproachStop: no pose on \"%s\" within %.2fs of onStart - stopping rather "
        "than creep on an unknown heading",
        pose_topic_.c_str(), scan_loss_timeout_);
      return BT::NodeStatus::FAILURE;
    }
    box_pose = last_box_pose_;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, global_frame_, robot_base_frame_)) {
    publishZero();
    RCLCPP_ERROR(
      node_->get_logger(),
      "FinalApproachStop: TF lookup failed (\"%s\" <- \"%s\") - stopping rather than "
      "creep on a stale heading",
      global_frame_.c_str(), robot_base_frame_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double dx = box_pose.pose.position.x - robot_pose.pose.position.x;
  const double dy = box_pose.pose.position.y - robot_pose.pose.position.y;
  const double bearing_to_box = std::atan2(dy, dx);
  const double bearing_error = normalizeAngle(bearing_to_box - robot_yaw);

  // g8: the forward lidar sector can go fully blind at contact range (e.g.
  // self-occluded by the manipulator arm), so the range-based stop below can
  // never fire. This test is independent of the lidar and uses only the
  // frozen box position (poseCallback) plus the current TF robot pose, so it
  // still stops the robot when every forward beam reads "no return".
  const double center_distance = std::hypot(dx, dy);
  if (center_distance <= stop_center_distance_) {
    publishZero();
    RCLCPP_INFO(
      node_->get_logger(),
      "FinalApproachStop: pose-based stop fired, box-centre distance %.2fm <= %.2fm",
      center_distance, stop_center_distance_);
    return BT::NodeStatus::SUCCESS;
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
    // r <= 0 is a no-return sentinel, not a distance: Isaac's RTX lidar sends
    // -1.0 and the TurtleBot3 LDS-01 driver 0.0 (g3 stopped on a -1.0 read as
    // "0 m ahead"). A positive reading below range_min means "closer than the
    // sensor can measure" - clamp to 0 so it still drives the stop.
    if (r <= 0.0) {
      continue;
    }
    min_range = std::min(min_range, r < scan.range_min ? 0.0 : r);
  }

  if (std::isfinite(min_range) && min_range <= stop_distance_) {
    publishZero();
    RCLCPP_INFO(
      node_->get_logger(), "FinalApproachStop: lidar stop fired, min forward range %.2fm <= %.2fm",
      min_range, stop_distance_);
    return BT::NodeStatus::SUCCESS;
  }

  const double angular = std::clamp(
    heading_gain_ * bearing_error, -max_angular_speed_, max_angular_speed_);
  if (std::abs(bearing_error) > align_threshold_) {
    // Outside the align cone: rotate in place rather than creep forward on
    // a heading that would (as in g6) miss the box out of the stop-check
    // cone entirely and drive straight past/into it from the side.
    publishTwist(0.0, angular);
  } else {
    publishTwist(approach_speed_, angular);
  }
  return BT::NodeStatus::RUNNING;
}

void FinalApproachStopAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "FinalApproachStop: halted");
  publishZero();
}

void FinalApproachStopAction::publishZero()
{
  publishTwist(0.0, 0.0);
}

void FinalApproachStopAction::publishTwist(double linear, double angular)
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_->publish(cmd);
  }
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::FinalApproachStopAction>("FinalApproachStop");
}
