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

#include "custom_nav2_bt_plugins/wiggle_in_place_action.hpp"

#include <algorithm>
#include <cmath>

namespace custom_nav2_bt_plugins
{

namespace
{
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

WiggleInPlaceAction::WiggleInPlaceAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME),
  last_log_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus WiggleInPlaceAction::onStart()
{
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("odom_topic", odom_topic_);
  getInput("angle", angle_);
  getInput("angular_speed", angular_speed_);
  getInput("yaw_tolerance", yaw_tolerance_);
  getInput("time_allowance", time_allowance_);

  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != cmd_vel_topic_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SystemDefaultsQoS());
  }
  if (!odom_sub_ || odom_sub_->get_topic_name() != odom_topic_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    latest_odom_.reset();
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {latest_odom_ = msg;},
      opts);
  }

  phase_ = Phase::WAIT_FOR_ODOM;
  start_time_ = node_->now();
  last_log_time_ = start_time_;
  RCLCPP_INFO(
    node_->get_logger(), "WiggleInPlace: sweeping +/-%.2f rad to help AMCL reconverge", angle_);
  return BT::NodeStatus::RUNNING;
}

bool WiggleInPlaceAction::rotateToward(double target_yaw)
{
  const double current_yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
  const double yaw_error = normalizeAngle(target_yaw - current_yaw);

  if (std::abs(yaw_error) <= yaw_tolerance_) {
    return true;
  }

  geometry_msgs::msg::Twist cmd;
  // was a plain k=1.0 P-controller (same formula DriveToRecordedPose's
  // CORRECT_YAW phase uses) - timed out completely (15s, full budget, no
  // partial progress logged) driving a much larger, previously-untested
  // pure-rotation-only command (that phase always had translation alongside
  // it). Best guess: the commanded angular.z was too small to overcome
  // real static friction/deadband for an in-place spin with zero linear
  // component. Floor the magnitude so it's never a weak command, and bump
  // the gain so it saturates at angular_speed_ sooner instead of easing in.
  const double k = 2.0;
  const double min_cmd = 0.15;
  double cmd_z = std::clamp(k * yaw_error, -angular_speed_, angular_speed_);
  if (std::abs(cmd_z) < min_cmd) {
    cmd_z = std::copysign(min_cmd, yaw_error);
  }
  cmd.angular.z = cmd_z;
  cmd_vel_pub_->publish(cmd);
  return false;
}

BT::NodeStatus WiggleInPlaceAction::onRunning()
{
  callback_group_executor_.spin_some();

  if ((node_->now() - start_time_).seconds() > time_allowance_) {
    publishZero();
    RCLCPP_ERROR(node_->get_logger(), "WiggleInPlace: time_allowance exceeded");
    return BT::NodeStatus::FAILURE;
  }

  if (!latest_odom_) {
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAIT_FOR_ODOM) {
    start_yaw_ = yawFromQuaternion(latest_odom_->pose.pose.orientation);
    phase_ = Phase::SWEEP_POS;
    RCLCPP_INFO(node_->get_logger(), "WiggleInPlace: -> SWEEP_POS (start_yaw=%.3f)", start_yaw_);
  }

  // Periodic status line (~1Hz) so a timeout shows exactly which phase and
  // how much yaw error remained, instead of just "time_allowance exceeded".
  if ((node_->now() - last_log_time_).seconds() > 1.0) {
    const double current_yaw = yawFromQuaternion(latest_odom_->pose.pose.orientation);
    RCLCPP_INFO(
      node_->get_logger(), "WiggleInPlace: phase=%d current_yaw=%.3f",
      static_cast<int>(phase_), current_yaw);
    last_log_time_ = node_->now();
  }

  bool reached = false;
  switch (phase_) {
    case Phase::SWEEP_POS:
      reached = rotateToward(normalizeAngle(start_yaw_ + angle_));
      if (reached) {
        phase_ = Phase::SWEEP_NEG;
        RCLCPP_INFO(node_->get_logger(), "WiggleInPlace: -> SWEEP_NEG");
      }
      break;
    case Phase::SWEEP_NEG:
      reached = rotateToward(normalizeAngle(start_yaw_ - angle_));
      if (reached) {
        phase_ = Phase::RETURN;
        RCLCPP_INFO(node_->get_logger(), "WiggleInPlace: -> RETURN");
      }
      break;
    case Phase::RETURN:
      reached = rotateToward(start_yaw_);
      if (reached) {
        publishZero();
        RCLCPP_INFO(node_->get_logger(), "WiggleInPlace: done");
        return BT::NodeStatus::SUCCESS;
      }
      break;
    default:
      break;
  }
  return BT::NodeStatus::RUNNING;
}

void WiggleInPlaceAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "WiggleInPlace: halted");
  publishZero();
}

void WiggleInPlaceAction::publishZero()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
  }
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::WiggleInPlaceAction>("WiggleInPlace");
}
