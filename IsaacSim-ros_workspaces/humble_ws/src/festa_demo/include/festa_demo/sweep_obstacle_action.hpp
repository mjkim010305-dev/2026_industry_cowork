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

#ifndef FESTA_DEMO__SWEEP_OBSTACLE_ACTION_HPP_
#define FESTA_DEMO__SWEEP_OBSTACLE_ACTION_HPP_

#include <future>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "festa_demo/action/sweep.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace festa_demo
{

/**
 * @brief Ask the manipulator side to clear the obstacle in front of the robot
 *        via the /sweep action (festa_demo/action/Sweep).
 *
 * The server is festa_manipulation/festa_action/sweep_action_server.py,
 * which runs obstacle_clear_sequence.py all: rear place -> sweep -> retreat
 * -> rear re-pick, ending back at P_REAR_CARRY. The arm must already be at
 * P_REAR_CARRY when this node starts (see RearPickObject).
 *
 * SUCCESS only when the action succeeds AND result_code == "SUCCESS". Any
 * other outcome (ERROR, STALL, goal rejected because a sweep is already
 * running, server missing, sweep_timeout) returns FAILURE; the reason is
 * written to `result_code` either way.
 *
 * Why not nav2_behavior_tree::BtActionNode: its constructor waits for the
 * action server and throws if it is not up, which would make the whole BT
 * fail to load whenever the manipulator side is not running. This node
 * creates its client lazily and only fails itself.
 *
 * The server rejects cancel requests (its sequence has no safe external
 * stop), so halting this node does NOT stop the arm - it only stops waiting.
 */
class SweepObstacleAction : public BT::StatefulActionNode
{
public:
  using Sweep = festa_demo::action::Sweep;
  using SweepGoalHandle = rclcpp_action::ClientGoalHandle<Sweep>;

  SweepObstacleAction(const std::string & name, const BT::NodeConfiguration & conf);
  SweepObstacleAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "action_name", std::string("/sweep"), "Sweep action server name"),
      BT::InputPort<double>(
        "server_timeout", 5.0, "Seconds to wait for the /sweep action server"),
      BT::InputPort<double>(
        "accept_timeout", 5.0, "Seconds to wait for the goal to be accepted"),
      BT::InputPort<double>(
        "sweep_timeout", 120.0,
        "Seconds to wait for the result before failing (<= 0 waits forever)"),
      BT::OutputPort<std::string>(
        "result_code",
        "SUCCESS / STALL / ERROR from the server, or REJECTED / NO_SERVER / TIMEOUT"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { WAIT_SERVER, WAIT_ACCEPT, WAIT_RESULT };

  BT::NodeStatus finish(const std::string & result_code, bool success);
  void cancelActive();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp_action::Client<Sweep>::SharedPtr client_;
  std::string action_name_;
  double server_timeout_ {5.0};
  double accept_timeout_ {5.0};
  double sweep_timeout_ {120.0};

  Phase phase_ {Phase::WAIT_SERVER};
  std::string last_state_;

  rclcpp::Time start_time_;
  rclcpp::Time phase_start_time_;

  std::shared_future<SweepGoalHandle::SharedPtr> goal_future_;
  SweepGoalHandle::SharedPtr goal_handle_;
  std::shared_future<SweepGoalHandle::WrappedResult> result_future_;
};

}  // namespace festa_demo

#endif  // FESTA_DEMO__SWEEP_OBSTACLE_ACTION_HPP_
