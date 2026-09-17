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

#ifndef CUSTOM_NAV2_BT_PLUGINS__RETRACT_ARM_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__RETRACT_ARM_ACTION_HPP_

#include <array>
#include <cstddef>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Retract the arm from PushObstacle's "push" pose back to home.
 *
 * Companion to PushObstacleAction: run this AFTER the base has driven
 * forward (e.g. via DriveOnHeading) while the arm was held rigid at the
 * push pose. Uses the same joint poses as PushObstacleAction (pre_push /
 * home) so the arm retraces the same path back. Arm-only, no gripper
 * action (the gripper was already left open by PushObstacle).
 *
 * Steps: pre_push -> home
 *
 * Non-blocking: StatefulActionNode with an internal step machine, same
 * pattern as FixedPickObjectAction / PlaceObjectAction / PushObstacleAction.
 */
class RetractArmAction : public BT::StatefulActionNode
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  RetractArmAction(const std::string & name, const BT::NodeConfiguration & conf);
  RetractArmAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "arm_action_name", std::string("/arm_controller/follow_joint_trajectory"),
        "FollowJointTrajectory action server for the arm"),
      BT::InputPort<std::string>(
        "arm_joint_names", std::string("joint1;joint2;joint3;joint4"),
        "Arm joint names, ';'-separated, in trajectory order"),
      BT::InputPort<double>("arm_move_time", 3.0, "Seconds allotted per arm move"),
      BT::InputPort<double>(
        "server_timeout", 5.0, "Seconds to wait for the action server"),
      BT::InputPort<double>(
        "step_timeout", 15.0, "Per-step timeout [s] before the retract fails"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { SEND, WAIT_ACCEPT, WAIT_RESULT };

  struct Step
  {
    std::array<double, 4> arm_pose;
    const char * label;
  };

  void buildSequence();
  BT::NodeStatus fail(const char * why);
  void cancelActive();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;

  std::string arm_action_name_;
  std::vector<std::string> arm_joint_names_;
  double arm_move_time_ {3.0};
  double server_timeout_ {5.0};
  double step_timeout_ {15.0};

  std::vector<Step> steps_;
  std::size_t step_idx_ {0};
  Phase phase_ {Phase::SEND};
  bool servers_ready_ {false};

  rclcpp::Time start_time_;
  rclcpp::Time step_start_time_;

  std::shared_future<ArmGoalHandle::SharedPtr> arm_goal_future_;
  ArmGoalHandle::SharedPtr arm_goal_handle_;
  std::shared_future<ArmGoalHandle::WrappedResult> arm_result_future_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__RETRACT_ARM_ACTION_HPP_
