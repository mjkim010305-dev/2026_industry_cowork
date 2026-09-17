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

#include "custom_nav2_bt_plugins/place_object_action.hpp"

#include <algorithm>
#include <chrono>
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
}  // namespace

PlaceObjectAction::PlaceObjectAction(
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
}

void PlaceObjectAction::buildSequence()
{
  // Joint values ported verbatim from fixed_pick_place.py (joint1..joint4).
  // P_PRE_PLACE == P_PRE_PICK and P_PLACE == P_PICK there: the object is set
  // down directly in front of wherever the base currently is - same arm
  // geometry as picking it up. Only the base pose (handled upstream, e.g.
  // SetFixedGoal + ComputePathToPose/FollowPath) differs between the pick and
  // place locations.
  const std::array<double, 4> P_PRE_PLACE {{-0.019941750242513337 - 1.5707963268, 0.8375535101855601,
      -0.12578642460662257, -0.8436894333371027}};
  const std::array<double, 4> P_PLACE     {{-0.02454369260617026 - 1.5707963268, 1.2885438618239387,
      -0.7731263170943632, -0.4816699673960913}};
  // Same as FixedPickObjectAction's P_LIFT: a straight-up retreat from
  // P_PICK/P_PLACE (same joint1). Without this, going straight from
  // gripper_open back to P_PRE_PLACE swept the open gripper back through the
  // space the just-released object now occupies, knocking it over.
  const std::array<double, 4> P_LIFT      {{-0.02454369260617026 - 1.5707963268, 0.518485063053467,
      0.04908738521234052, -0.6181942575179133}};
  const std::array<double, 4> P_HOME      {{-0.0015339807878856412 - 1.5707963268, -1.0461748973380072,
      1.0753205323078345, 0.009203884727313847}};

  const std::array<double, 4> none {{0.0, 0.0, 0.0, 0.0}};

  steps_.clear();
  steps_.push_back({Kind::ARM,     P_PRE_PLACE, 0.0,            0.0, "pre_place"});
  steps_.push_back({Kind::ARM,     P_PLACE,     0.0,            0.0, "place"});
  steps_.push_back(
    {Kind::GRIPPER, none, open_position_, post_open_delay_, "gripper_open"});
  steps_.push_back({Kind::ARM,     P_LIFT,      0.0,            0.0, "lift"});
  steps_.push_back({Kind::ARM,     P_PRE_PLACE, 0.0,            0.0, "back_to_pre_place"});
  steps_.push_back({Kind::ARM,     P_HOME,      0.0,            0.0, "home"});
}

BT::NodeStatus PlaceObjectAction::onStart()
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
  getInput("max_effort", max_effort_);
  getInput("server_timeout", server_timeout_);
  getInput("step_timeout", step_timeout_);
  getInput("post_open_delay", post_open_delay_);

  if (arm_joint_names_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "PlaceObject: arm_joint_names is empty");
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
  phase_ = Phase::SEND;
  servers_ready_ = false;
  arm_goal_handle_.reset();
  grip_goal_handle_.reset();
  start_time_ = node_->now();
  step_start_time_ = start_time_;

  RCLCPP_INFO(
    node_->get_logger(),
    "PlaceObject: starting fixed place sequence (%zu steps)", steps_.size());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceObjectAction::onRunning()
{
  callback_group_executor_.spin_some();

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
        node_->get_logger(), "PlaceObject: step %zu/%zu '%s' sent",
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
            node_->get_logger(), "PlaceObject: step '%s' did not succeed", step.label);
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
          node_->get_logger(), "PlaceObject: step %zu/%zu '%s' done",
          step_idx_ + 1, steps_.size(), step.label);
        step_idx_++;
        phase_ = Phase::SEND;
        step_start_time_ = node_->now();
        if (step_idx_ >= steps_.size()) {
          RCLCPP_INFO(node_->get_logger(), "PlaceObject: place sequence complete");
          return BT::NodeStatus::SUCCESS;
        }
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void PlaceObjectAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "PlaceObject: halted, cancelling any active goal");
  cancelActive();
}

BT::NodeStatus PlaceObjectAction::fail(const char * why)
{
  RCLCPP_ERROR(node_->get_logger(), "PlaceObject: %s", why);
  cancelActive();
  return BT::NodeStatus::FAILURE;
}

void PlaceObjectAction::cancelActive()
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
  factory.registerNodeType<custom_nav2_bt_plugins::PlaceObjectAction>("PlaceObject");
}
