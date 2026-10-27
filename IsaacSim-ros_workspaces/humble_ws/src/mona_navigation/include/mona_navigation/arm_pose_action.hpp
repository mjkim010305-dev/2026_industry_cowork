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

#ifndef MONA_NAVIGATION__ARM_POSE_ACTION_HPP_
#define MONA_NAVIGATION__ARM_POSE_ACTION_HPP_

#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace mona_navigation
{

/**
 * @brief Move the arm to a joint pose via FollowJointTrajectory. Sends two points (current
 * /joint_states at t=0, target at move_time) because the Isaac bridge snaps single-point
 * targets. SUCCESS on error_code SUCCESSFUL; FAILURE on reject / timeout / abort / no joint
 * state. Halt cancels the goal. Client and subscription are created lazily.
 */
class ArmPose : public BT::StatefulActionNode
{
public:
  using Fjt = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Fjt>;

  ArmPose(const std::string & name, const BT::NodeConfiguration & conf);
  ArmPose() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "action_name", std::string("/arm_controller/follow_joint_trajectory"),
        "FollowJointTrajectory action name"),
      BT::InputPort<std::string>("positions", "Target joint positions \"j1;j2;j3;j4\" [rad]"),
      BT::InputPort<std::string>(
        "joint_names", std::string("joint1;joint2;joint3;joint4"), "Joint names"),
      BT::InputPort<double>("move_time", 2.0, "Seconds for the move"),
      BT::InputPort<double>("server_timeout", 5.0, "Seconds to wait for server / acceptance"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { WAIT_SERVER, WAIT_ACCEPT, WAIT_RESULT };

  void cancelActive();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp_action::Client<Fjt>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_;

  std::string action_name_;
  std::vector<std::string> names_;
  std::vector<double> target_;
  double move_time_ {2.0};
  double server_timeout_ {5.0};

  Phase phase_ {Phase::WAIT_SERVER};
  rclcpp::Time phase_start_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__ARM_POSE_ACTION_HPP_
