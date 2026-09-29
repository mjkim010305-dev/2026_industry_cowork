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

#include "custom_nav2_bt_plugins/rear_pick_object_action.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace custom_nav2_bt_plugins
{

namespace
{
template<typename FutureT>
bool ready(const FutureT & f)
{
  return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

// Joint poses copied from ~/turtlebot3_ws/scripts/festa_action/rear_pick.py
// (joint1..joint4). P_REAR_CARRY is also the start pose that
// obstacle_clear_sequence.py (the /sweep server's sequence) checks for.
const std::array<double, 4> P_HOME {{-0.0015339807878856412, -1.0461748973380072,
    1.0753205323078345, 0.009203884727313847}};
const std::array<double, 4> P_PRE_PICK {{-0.0030679615757712823, 0.7271068934577939,
    -0.48013598660820567, 0.2561747915769021}};
const std::array<double, 4> P_PICK {{-0.0030679615757712823, 0.923456434307156,
    -0.5706408530934585, 0.25770877236478773}};
const std::array<double, 4> P_LIFT {{-0.0030679615757712823, -0.04295146206079795,
    -0.13959225169759334, 0.2945243112740431}};
const std::array<double, 4> P_SIDE_CARRY {{-1.59073807703741, -0.035281558121369745,
    -0.13959225169759334, 0.2945243112740431}};
const std::array<double, 4> P_REAR_CARRY {{-3.0234761329225988, -0.9802137234589247,
    0.5016117176386047, 0.2193592526676467}};
}  // namespace

RearPickObjectAction::RearPickObjectAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME),
  step_start_time_(0, 0, RCL_ROS_TIME),
  delay_start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  std::string joint_states_topic = "/joint_states";
  getInput("joint_states_topic", joint_states_topic);

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS(),
    std::bind(&RearPickObjectAction::jointStateCallback, this, std::placeholders::_1),
    sub_option);
}

void RearPickObjectAction::jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (arm_joint_names_.empty()) {
    return;
  }
  std::vector<double> pose;
  pose.reserve(arm_joint_names_.size());
  for (const auto & name : arm_joint_names_) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end()) {
      return;
    }
    const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size()) {
      return;
    }
    pose.push_back(msg->position[idx]);
  }
  current_pose_ = pose;
  have_joint_state_ = true;
}

void RearPickObjectAction::buildSequence()
{
  const std::array<double, 4> none {{0.0, 0.0, 0.0, 0.0}};

  steps_.clear();
  steps_.push_back({Kind::GRIPPER, none,         open_position_,  0.0, "gripper_open"});
  steps_.push_back({Kind::ARM,     P_HOME,       0.0,             0.0, "home"});
  steps_.push_back({Kind::ARM,     P_PRE_PICK,   0.0,             0.0, "pre_pick"});
  steps_.push_back({Kind::ARM,     P_PICK,       0.0,             0.0, "pick"});
  steps_.push_back({Kind::GRIPPER, none,         grasp_position_, grasp_settle_time_,
      "gripper_grasp"});
  steps_.push_back({Kind::ARM,     P_LIFT,       0.0,             0.0, "lift"});
  steps_.push_back({Kind::ARM,     P_SIDE_CARRY, 0.0,             0.0, "side_carry"});
  steps_.push_back({Kind::ARM,     P_REAR_CARRY, 0.0,             0.0, "rear_carry"});
}

BT::NodeStatus RearPickObjectAction::onStart()
{
  getInput("arm_action_name", arm_action_name_);
  getInput("gripper_action_name", gripper_action_name_);
  std::string joint_names_str;
  getInput("arm_joint_names", joint_names_str);
  const auto parts = BT::splitString(joint_names_str, ';');
  arm_joint_names_.clear();
  for (const auto & p : parts) {
    arm_joint_names_.push_back(std::string(p));
  }
  getInput("arm_move_time", arm_move_time_);
  getInput("open_position", open_position_);
  getInput("grasp_position", grasp_position_);
  getInput("max_effort", max_effort_);
  getInput("grasp_settle_time", grasp_settle_time_);
  getInput("carry_tolerance", carry_tolerance_);
  getInput("joint_state_timeout", joint_state_timeout_);
  getInput("server_timeout", server_timeout_);
  getInput("step_timeout", step_timeout_);

  if (arm_joint_names_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "RearPickObject: arm_joint_names is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!arm_client_) {
    arm_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      node_, arm_action_name_, callback_group_);
  }
  if (!gripper_client_) {
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      node_, gripper_action_name_, callback_group_);
  }

  buildSequence();
  step_idx_ = 0;
  phase_ = Phase::CHECK_CARRY;
  servers_ready_ = false;
  have_joint_state_ = false;
  current_pose_.clear();
  arm_goal_handle_.reset();
  grip_goal_handle_.reset();
  start_time_ = node_->now();
  step_start_time_ = start_time_;

  RCLCPP_INFO(
    node_->get_logger(),
    "RearPickObject: starting rear pick sequence (%zu steps)", steps_.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RearPickObjectAction::onRunning()
{
  callback_group_executor_.spin_some();

  if (phase_ == Phase::CHECK_CARRY) {
    if (have_joint_state_) {
      double max_err = 0.0;
      const std::size_t n = std::min<std::size_t>(current_pose_.size(), P_REAR_CARRY.size());
      for (std::size_t i = 0; i < n; ++i) {
        max_err = std::max(max_err, std::abs(current_pose_[i] - P_REAR_CARRY[i]));
      }
      if (max_err <= carry_tolerance_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "RearPickObject: arm already at P_REAR_CARRY (max err %.3f rad), skipping pick",
          max_err);
        return BT::NodeStatus::SUCCESS;
      }
    } else if ((node_->now() - start_time_).seconds() <= joint_state_timeout_) {
      return BT::NodeStatus::RUNNING;
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "RearPickObject: no joint state within %.1f s, running the full pick",
        joint_state_timeout_);
    }
    phase_ = Phase::SEND;
    start_time_ = node_->now();
  }

  if (!servers_ready_) {
    if (arm_client_->action_server_is_ready() && gripper_client_->action_server_is_ready()) {
      servers_ready_ = true;
      step_start_time_ = node_->now();
    } else if ((node_->now() - start_time_).seconds() > server_timeout_) {
      return fail("arm/gripper action server not available");
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  if (step_idx_ >= steps_.size()) {
    return BT::NodeStatus::SUCCESS;
  }

  const Step & step = steps_[step_idx_];
  const double step_age = (node_->now() - step_start_time_).seconds();

  switch (phase_) {
    case Phase::CHECK_CARRY:
      return BT::NodeStatus::RUNNING;

    case Phase::SEND: {
      if (step.kind == Kind::ARM) {
        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = arm_joint_names_;
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        const std::size_t n =
          std::min<std::size_t>(step.arm_pose.size(), arm_joint_names_.size());
        pt.positions.assign(
          step.arm_pose.begin(),
          step.arm_pose.begin() + static_cast<std::ptrdiff_t>(n));
        pt.time_from_start.sec = static_cast<int32_t>(arm_move_time_);
        pt.time_from_start.nanosec = static_cast<uint32_t>(
          (arm_move_time_ - static_cast<double>(pt.time_from_start.sec)) * 1e9);
        goal.trajectory.points.push_back(pt);
        arm_goal_future_ = arm_client_->async_send_goal(goal);
      } else {
        GripperCommand::Goal goal;
        goal.command.position = step.gripper_pos;
        goal.command.max_effort = max_effort_;
        grip_goal_future_ = gripper_client_->async_send_goal(goal);
      }
      step_start_time_ = node_->now();
      phase_ = Phase::WAIT_ACCEPT;
      RCLCPP_INFO(
        node_->get_logger(), "RearPickObject: step %zu/%zu '%s' sent",
        step_idx_ + 1, steps_.size(), step.label);
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_ACCEPT: {
      if (step.kind == Kind::ARM) {
        if (ready(arm_goal_future_)) {
          arm_goal_handle_ = arm_goal_future_.get();
          if (!arm_goal_handle_) {
            return fail("arm goal rejected by the controller");
          }
          arm_result_future_ = arm_client_->async_get_result(arm_goal_handle_);
          phase_ = Phase::WAIT_RESULT;
        }
      } else {
        if (ready(grip_goal_future_)) {
          grip_goal_handle_ = grip_goal_future_.get();
          if (!grip_goal_handle_) {
            return fail("gripper goal rejected by the controller");
          }
          grip_result_future_ = gripper_client_->async_get_result(grip_goal_handle_);
          phase_ = Phase::WAIT_RESULT;
        }
      }
      if (phase_ == Phase::WAIT_ACCEPT && step_age > step_timeout_) {
        return fail("timed out waiting for goal acceptance");
      }
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_RESULT: {
      bool done = false;
      bool ok = false;
      if (step.kind == Kind::ARM) {
        if (ready(arm_result_future_)) {
          const auto wrapped = arm_result_future_.get();
          done = true;
          ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
      } else {
        if (ready(grip_result_future_)) {
          const auto wrapped = grip_result_future_.get();
          done = true;
          ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
      }
      if (done) {
        arm_goal_handle_.reset();
        grip_goal_handle_.reset();
        if (!ok) {
          RCLCPP_ERROR(
            node_->get_logger(), "RearPickObject: step '%s' did not succeed", step.label);
          return BT::NodeStatus::FAILURE;
        }
        delay_start_time_ = node_->now();
        phase_ = Phase::POST_DELAY;
      } else if (step_age > step_timeout_) {
        return fail("timed out waiting for the step result");
      }
      return BT::NodeStatus::RUNNING;
    }

    case Phase::POST_DELAY: {
      if ((node_->now() - delay_start_time_).seconds() >= step.post_delay) {
        RCLCPP_INFO(
          node_->get_logger(), "RearPickObject: step %zu/%zu '%s' done",
          step_idx_ + 1, steps_.size(), step.label);
        step_idx_++;
        phase_ = Phase::SEND;
        step_start_time_ = node_->now();
        if (step_idx_ >= steps_.size()) {
          RCLCPP_INFO(node_->get_logger(), "RearPickObject: rear pick sequence complete");
          return BT::NodeStatus::SUCCESS;
        }
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void RearPickObjectAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "RearPickObject: halted, cancelling any active goal");
  cancelActive();
}

BT::NodeStatus RearPickObjectAction::fail(const char * why)
{
  RCLCPP_ERROR(node_->get_logger(), "RearPickObject: %s", why);
  cancelActive();
  return BT::NodeStatus::FAILURE;
}

void RearPickObjectAction::cancelActive()
{
  if (arm_goal_handle_ && arm_client_) {
    arm_client_->async_cancel_goal(arm_goal_handle_);
    arm_goal_handle_.reset();
  }
  if (grip_goal_handle_ && gripper_client_) {
    gripper_client_->async_cancel_goal(grip_goal_handle_);
    grip_goal_handle_.reset();
  }
  callback_group_executor_.spin_some();
}

}  // namespace custom_nav2_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<custom_nav2_bt_plugins::RearPickObjectAction>("RearPickObject");
}
