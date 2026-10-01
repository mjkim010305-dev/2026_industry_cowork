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

#include "festa_demo/sweep_obstacle_action.hpp"

#include <chrono>

namespace festa_demo
{

namespace
{
template<typename FutureT>
bool ready(const FutureT & f)
{
  return f.valid() && f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}
}  // namespace

SweepObstacleAction::SweepObstacleAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf),
  start_time_(0, 0, RCL_ROS_TIME),
  phase_start_time_(0, 0, RCL_ROS_TIME)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());
}

BT::NodeStatus SweepObstacleAction::onStart()
{
  getInput("action_name", action_name_);
  getInput("server_timeout", server_timeout_);
  getInput("accept_timeout", accept_timeout_);
  getInput("sweep_timeout", sweep_timeout_);

  if (!client_) {
    client_ = rclcpp_action::create_client<Sweep>(node_, action_name_, callback_group_);
  }

  phase_ = Phase::WAIT_SERVER;
  last_state_.clear();
  goal_handle_.reset();
  start_time_ = node_->now();
  phase_start_time_ = start_time_;

  RCLCPP_INFO(node_->get_logger(), "SweepObstacle: requesting sweep on \"%s\"",
    action_name_.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SweepObstacleAction::onRunning()
{
  callback_group_executor_.spin_some();

  const double phase_age = (node_->now() - phase_start_time_).seconds();

  switch (phase_) {
    case Phase::WAIT_SERVER: {
      if (!client_->action_server_is_ready()) {
        if (phase_age > server_timeout_) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "SweepObstacle: \"%s\" server not available after %.1f s "
            "(is sweep_action_server.py running?)", action_name_.c_str(), server_timeout_);
          return finish("NO_SERVER", false);
        }
        return BT::NodeStatus::RUNNING;
      }

      rclcpp_action::Client<Sweep>::SendGoalOptions options;
      options.feedback_callback =
        [this](SweepGoalHandle::SharedPtr, const std::shared_ptr<const Sweep::Feedback> fb) {
          if (fb->state != last_state_) {
            last_state_ = fb->state;
            RCLCPP_INFO(node_->get_logger(), "SweepObstacle: state -> %s", last_state_.c_str());
          }
        };
      goal_future_ = client_->async_send_goal(Sweep::Goal(), options);
      phase_ = Phase::WAIT_ACCEPT;
      phase_start_time_ = node_->now();
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_ACCEPT: {
      if (ready(goal_future_)) {
        goal_handle_ = goal_future_.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "SweepObstacle: goal rejected (a sweep is probably already running)");
          return finish("REJECTED", false);
        }
        result_future_ = client_->async_get_result(goal_handle_);
        phase_ = Phase::WAIT_RESULT;
        phase_start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "SweepObstacle: goal accepted, sweeping");
      } else if (phase_age > accept_timeout_) {
        // festa_demo: 2026-10-01 - under Pi load the "accepted" reply can
        // arrive well after we give up; the server may already have
        // accepted and started executing. Ask it to cancel everything (no
        // goal handle exists yet, so cancelActive() has nothing to act on)
        // and drop the future so a late reply is ignored.
        RCLCPP_ERROR(node_->get_logger(), "SweepObstacle: timed out waiting for goal acceptance");
        client_->async_cancel_all_goals();
        callback_group_executor_.spin_some();
        goal_future_ = std::shared_future<SweepGoalHandle::SharedPtr>();
        RCLCPP_WARN(
          node_->get_logger(),
          "SweepObstacle: sent cancel-all in case the server accepted after all");
        return finish("TIMEOUT", false);
      }
      return BT::NodeStatus::RUNNING;
    }

    case Phase::WAIT_RESULT: {
      if (ready(result_future_)) {
        const auto wrapped = result_future_.get();
        goal_handle_.reset();
        const std::string code = wrapped.result ? wrapped.result->result_code : std::string();
        const bool ok = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && code == "SUCCESS";
        if (ok) {
          RCLCPP_INFO(node_->get_logger(), "SweepObstacle: sweep complete (SUCCESS)");
        } else {
          RCLCPP_ERROR(
            node_->get_logger(),
            "SweepObstacle: sweep failed, result_code=\"%s\" (last state \"%s\")",
            code.c_str(), last_state_.c_str());
        }
        return finish(code.empty() ? std::string("ERROR") : code, ok);
      }
      if (sweep_timeout_ > 0.0 && phase_age > sweep_timeout_) {
        RCLCPP_ERROR(
          node_->get_logger(),
          "SweepObstacle: no result within %.1f s (last state \"%s\") - the arm may "
          "still be moving on the manipulator side", sweep_timeout_, last_state_.c_str());
        cancelActive();
        return finish("TIMEOUT", false);
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void SweepObstacleAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "SweepObstacle: halted during sweep - the server does not support cancel, "
      "the arm sequence keeps running on the manipulator side");
  }
  cancelActive();
}

BT::NodeStatus SweepObstacleAction::finish(const std::string & result_code, bool success)
{
  setOutput("result_code", result_code);
  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SweepObstacleAction::cancelActive()
{
  if (goal_handle_ && client_) {
    client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  callback_group_executor_.spin_some();
}

}  // namespace festa_demo

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<festa_demo::SweepObstacleAction>("SweepObstacle");
}
