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

#include "mona_navigation/creep_to_face_action.hpp"

#include <chrono>

#include "tf2/exceptions.h"

namespace mona_navigation
{

CreepToFaceAction::CreepToFaceAction(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

CreepToFaceAction::~CreepToFaceAction()
{
  if (pub_) {
    stop();
  }
}

BT::NodeStatus CreepToFaceAction::onStart()
{
  geometry_msgs::msg::PoseStamped target;
  if (!getInput("target", target)) {
    RCLCPP_ERROR(node_->get_logger(), "CreepToFace: no target");
    return BT::NodeStatus::FAILURE;
  }
  std::string cmd_vel_topic, odom_topic;
  getInput("speed", speed_);
  getInput("timeout", timeout_);
  getInput("stall_time", stall_time_);
  getInput("stall_ratio", stall_ratio_);
  getInput("yaw_tol", yaw_tol_);
  getInput("cmd_vel_topic", cmd_vel_topic);
  getInput("odom_topic", odom_topic);
  getInput("base_frame", base_frame_);
  getInput("global_frame", global_frame_);
  rotate_only_ = false;
  getInput("rotate_only", rotate_only_);
  max_travel_ = 0.0;
  getInput("max_travel", max_travel_);

  if (!target.header.frame_id.empty() && target.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "CreepToFace: target frame \"%s\" != global_frame \"%s\"",
      target.header.frame_id.c_str(), global_frame_.c_str());
    setOutput("result", std::string("TF_ERROR"));
    return BT::NodeStatus::FAILURE;
  }
  target_x_ = target.pose.position.x;
  target_y_ = target.pose.position.y;
  target_yaw_ = detail::yaw_from_quat(target.pose.orientation);

  // Lazy (re)creation so the tree loads without the topics and port overrides are honored.
  if (!pub_ || cmd_vel_topic != cmd_vel_topic_) {
    cmd_vel_topic_ = cmd_vel_topic;
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }
  if (!sub_ || odom_topic != odom_topic_) {
    odom_topic_ = odom_topic;
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_speed_ = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
        travel_.update(m->pose.pose.position.x, m->pose.pose.position.y);
      },
      opts);
  }

  phase_ = Phase::ROTATE;
  start_s_ = node_->now().seconds();
  last_cmd_v_ = 0.0;
  odom_speed_ = 0.0;
  tf_failing_ = false;
  travel_.reset();
  stall_.reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CreepToFaceAction::onRunning()
{
  callback_group_executor_.spin_all(std::chrono::milliseconds(5));
  const double now_s = node_->now().seconds();

  if (now_s - start_s_ > timeout_) {
    return fail("timeout", tf_failing_ ? "TF_ERROR" : "TIMEOUT");
  }

  double x, y, yaw;
  try {
    const auto tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    yaw = detail::yaw_from_quat(tf.transform.rotation);
    tf_failing_ = false;
  } catch (const tf2::TransformException & e) {
    tf_failing_ = true;
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "CreepToFace: TF %s", e.what());
    stop();
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::Twist cmd;
  if (phase_ == Phase::ROTATE) {
    if (std::fabs(detail::wrap_angle(target_yaw_ - yaw)) < yaw_tol_) {
      if (rotate_only_) {
        return succeed("rotated to target heading");
      }
      phase_ = Phase::DRIVE;
      stall_.reset();
      travel_.reset();
    } else {
      cmd.angular.z = detail::rotate_cmd(yaw, target_yaw_);
      last_cmd_v_ = 0.0;
      pub_->publish(cmd);
      return BT::NodeStatus::RUNNING;
    }
  }

  if (detail::along_remaining(x, y, target_x_, target_y_, target_yaw_) <= 0.0) {
    return succeed("reached face");
  }

  if (max_travel_ > 0.0 && travel_.total >= max_travel_) {
    return succeed("max_travel reached");
  }

  if (stall_.update(now_s, last_cmd_v_, odom_speed_, stall_ratio_, stall_time_)) {
    return fail("stalled (blocked)", "STALLED");
  }

  cmd.linear.x = speed_;
  cmd.angular.z = detail::drive_cmd(
    yaw, target_yaw_, detail::lateral_error(x, y, target_x_, target_y_, target_yaw_));
  last_cmd_v_ = speed_;
  pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void CreepToFaceAction::onHalted()
{
  stop();
}

void CreepToFaceAction::stop()
{
  last_cmd_v_ = 0.0;
  if (pub_) {
    pub_->publish(geometry_msgs::msg::Twist());
  }
}

BT::NodeStatus CreepToFaceAction::succeed(const char * msg)
{
  stop();
  RCLCPP_INFO(node_->get_logger(), "CreepToFace: %s", msg);
  setOutput("result", std::string("REACHED"));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CreepToFaceAction::fail(const char * reason, const char * result)
{
  stop();
  setOutput("result", std::string(result));
  RCLCPP_WARN(node_->get_logger(), "CreepToFace: FAILURE, %s", reason);
  return BT::NodeStatus::FAILURE;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::CreepToFaceAction>("CreepToFace");
}
