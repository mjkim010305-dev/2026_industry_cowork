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
#include <utility>

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

  side_clear_stop_ = false;
  getInput("side_clear_stop", side_clear_stop_);
  if (side_clear_stop_) {
    side_half_ = 0.35;
    mona_interfaces::msg::PathObstacle obs;
    if (getInput("obstacle", obs) && obs.width_m > 0.0f) {
      side_half_ = 0.5 * obs.width_m;
    }
    getInput("side_width", side_width_);
    getInput("side_margin", side_margin_);
    getInput("side_along_min", side_along_min_);
    getInput("side_along_len", side_along_len_);
    getInput("lethal", lethal_);
    getInput("min_travel", min_travel_);
    std::string costmap_topic = "/local_costmap/costmap_raw";
    getInput("costmap_topic", costmap_topic);
    if (!costmap_sub_ || costmap_topic != costmap_topic_) {
      costmap_topic_ = costmap_topic;
      grid_ = detail::CostGrid();
      costmap_sub_ = subscribeCostmap(costmap_topic_, grid_, grid_frame_);
    }
    std::string global_topic = "/global_costmap/costmap_raw";
    getInput("global_costmap_topic", global_topic);
    if (global_topic != global_costmap_topic_ || (!global_topic.empty() && !global_costmap_sub_)) {
      global_costmap_topic_ = global_topic;
      global_grid_ = detail::CostGrid();
      global_costmap_sub_.reset();
      if (!global_costmap_topic_.empty()) {
        global_costmap_sub_ = subscribeCostmap(global_costmap_topic_, global_grid_, global_grid_frame_);
      }
    }
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

  const char * side = nullptr;
  if (side_clear_stop_ && travel_.total >= min_travel_ && sideClear(side)) {
    stop();
    RCLCPP_INFO(node_->get_logger(), "CreepToFace: side clear (%s)", side);
    setOutput("result", std::string("SIDE_CLEAR"));
    setOutput("clear_side", std::string(side));
    return BT::NodeStatus::SUCCESS;
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

rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr CreepToFaceAction::subscribeCostmap(
  const std::string & topic, detail::CostGrid & grid, std::string & frame)
{
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = callback_group_;
  return node_->create_subscription<nav2_msgs::msg::Costmap>(
    topic, rclcpp::QoS(1).reliable(),
    [&grid, &frame](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
      detail::CostGrid g;
      g.size_x = static_cast<int>(m->metadata.size_x);
      g.size_y = static_cast<int>(m->metadata.size_y);
      g.res = m->metadata.resolution;
      g.ox = m->metadata.origin.position.x;
      g.oy = m->metadata.origin.position.y;
      g.data = m->data;
      grid = std::move(g);
      frame = m->header.frame_id;
    },
    opts);
}

bool CreepToFaceAction::robotPose(const std::string & frame, detail::Pose2D & robot)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
    robot.x = tf.transform.translation.x;
    robot.y = tf.transform.translation.y;
    robot.yaw = detail::yaw_from_quat(tf.transform.rotation);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "CreepToFace: costmap TF %s", e.what());
    return false;
  }
  return true;
}

bool CreepToFaceAction::sideClear(const char *& side)
{
  // Callbacks run in callback_group_executor_.spin_all (onRunning); no concurrent access.
  detail::Pose2D robot, global_robot;
  if (!grid_.valid() || !robotPose(grid_frame_, robot)) {
    return false;
  }
  // The local costmap has no static layer, so racks open at lidar height (legs only) look free:
  // the window must also be clear in the global costmap (s36o).
  const bool use_global = !global_costmap_topic_.empty();
  if (use_global && (!global_grid_.valid() || !robotPose(global_grid_frame_, global_robot))) {
    return false;
  }
  const auto lethal = static_cast<uint8_t>(lethal_);
  for (const bool left : {true, false}) {
    if (detail::side_window_clear(
        grid_, robot, side_half_, side_margin_, side_width_, side_along_min_, side_along_len_,
        lethal, left) &&
      (!use_global || detail::side_window_clear(
        global_grid_, global_robot, side_half_, side_margin_, side_width_, side_along_min_,
        side_along_len_, lethal, left)))
    {
      side = left ? "left" : "right";
      return true;
    }
  }
  return false;
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
