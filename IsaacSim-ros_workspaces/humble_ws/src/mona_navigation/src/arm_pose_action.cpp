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

#include "mona_navigation/arm_pose_action.hpp"

#include <chrono>

#include "mona_navigation/push_geometry.hpp"

namespace mona_navigation
{

namespace
{
template<typename FutureT>
bool ready(const FutureT & f)
{
  return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}  // namespace

ArmPose::ArmPose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf), phase_start_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus ArmPose::onStart()
{
  std::string action_name, positions, joint_names;
  getInput("action_name", action_name);
  getInput("positions", positions);
  getInput("joint_names", joint_names);
  getInput("move_time", move_time_);
  getInput("server_timeout", server_timeout_);

  names_ = detail::split_semicolon(joint_names);
  target_.clear();
  try {
    for (const auto & s : detail::split_semicolon(positions)) {target_.push_back(std::stod(s));}
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "ArmPose: bad positions \"%s\"", positions.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (names_.empty() || names_.size() != target_.size()) {
    RCLCPP_ERROR(
      node_->get_logger(), "ArmPose: %zu joint names vs %zu positions",
      names_.size(), target_.size());
    return BT::NodeStatus::FAILURE;
  }

  if (!client_ || action_name != action_name_) {
    action_name_ = action_name;
    client_ = rclcpp_action::create_client<Fjt>(node_, action_name_, callback_group_);
  }
  if (!sub_) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {latest_ = m;}, opts);
  }

  phase_ = Phase::WAIT_SERVER;
  goal_handle_.reset();
  phase_start_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmPose::onRunning()
{
  callback_group_executor_.spin_all(std::chrono::milliseconds(20));
  const double age = (node_->now() - phase_start_).seconds();

  switch (phase_) {
    case Phase::WAIT_SERVER: {
      if (!client_->action_server_is_ready() || !latest_) {
        if (age > server_timeout_) {
          RCLCPP_ERROR(
            node_->get_logger(), "ArmPose: no server \"%s\" or /joint_states after %.1f s",
            action_name_.c_str(), server_timeout_);
          return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
      }
      std::vector<double> current;
      for (const auto & n : names_) {
        bool found = false;
        for (size_t i = 0; i < latest_->name.size() && i < latest_->position.size(); ++i) {
          if (latest_->name[i] == n) {
            current.push_back(latest_->position[i]);
            found = true;
            break;
          }
        }
        if (!found) {
          RCLCPP_ERROR(node_->get_logger(), "ArmPose: joint \"%s\" not in /joint_states", n.c_str());
          return BT::NodeStatus::FAILURE;
        }
      }
      Fjt::Goal goal;
      goal.trajectory = detail::two_point_trajectory(names_, current, target_, move_time_);
      goal_future_ = client_->async_send_goal(goal);
      phase_ = Phase::WAIT_ACCEPT;
      phase_start_ = node_->now();
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_ACCEPT: {
      if (ready(goal_future_)) {
        goal_handle_ = goal_future_.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(node_->get_logger(), "ArmPose: goal rejected");
          return BT::NodeStatus::FAILURE;
        }
        result_future_ = client_->async_get_result(goal_handle_);
        phase_ = Phase::WAIT_RESULT;
        phase_start_ = node_->now();
      } else if (age > server_timeout_) {
        RCLCPP_ERROR(node_->get_logger(), "ArmPose: timed out waiting for goal acceptance");
        client_->async_cancel_all_goals();
        callback_group_executor_.spin_all(std::chrono::milliseconds(20));
        goal_future_ = std::shared_future<GoalHandle::SharedPtr>();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_RESULT: {
      if (ready(result_future_)) {
        const auto wrapped = result_future_.get();
        goal_handle_.reset();
        const bool ok = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
          wrapped.result && wrapped.result->error_code == Fjt::Result::SUCCESSFUL;
        if (!ok) {
          RCLCPP_ERROR(node_->get_logger(), "ArmPose: trajectory failed (aborted/canceled/error)");
        }
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      if (age > move_time_ + server_timeout_) {
        RCLCPP_ERROR(node_->get_logger(), "ArmPose: no result within %.1f s", age);
        cancelActive();
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void ArmPose::onHalted()
{
  cancelActive();
}

void ArmPose::cancelActive()
{
  if (goal_handle_ && client_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  callback_group_executor_.spin_all(std::chrono::milliseconds(20));
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::ArmPose>("ArmPose");
}
