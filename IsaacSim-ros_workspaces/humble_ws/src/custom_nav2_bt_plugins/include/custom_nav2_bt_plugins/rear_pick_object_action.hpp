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

#ifndef CUSTOM_NAV2_BT_PLUGINS__REAR_PICK_OBJECT_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__REAR_PICK_OBJECT_ACTION_HPP_

#include <array>
#include <cstddef>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Pick the object in front of the robot and stow it behind the robot.
 *
 * BT port of ~/turtlebot3_ws/scripts/festa_action/rear_pick.py, same joint
 * poses. It ends at P_REAR_CARRY, which is the start pose the manipulator-side
 * /sweep sequence (obstacle_clear_sequence.py) checks for, so this is the pick
 * to use in front of a SweepObstacle node.
 *
 * Steps: gripper open -> home -> pre_pick -> pick -> gripper grasp -> lift
 *        -> side_carry -> rear_carry
 *
 * Unlike rear_pick.py it does not require the arm to start at P_HOME - it
 * moves there first, like FixedPickObject. If /joint_states shows the arm is
 * already at P_REAR_CARRY (within `carry_tolerance`), it returns SUCCESS
 * without moving: the robot is already carrying, and opening the gripper
 * would drop the object behind the robot (e.g. when a new NavigateToPose goal
 * restarts the tree).
 *
 * Non-blocking: StatefulActionNode with an internal step machine, same
 * pattern as FixedPickObjectAction / PushObstacleAction.
 */
class RearPickObjectAction : public BT::StatefulActionNode
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GripperCommand = control_msgs::action::GripperCommand;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GripGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  RearPickObjectAction(const std::string & name, const BT::NodeConfiguration & conf);
  RearPickObjectAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "arm_action_name", std::string("/arm_controller/follow_joint_trajectory"),
        "FollowJointTrajectory action server for the arm"),
      BT::InputPort<std::string>(
        "gripper_action_name", std::string("/gripper_controller/gripper_cmd"),
        "GripperCommand action server for the gripper"),
      BT::InputPort<std::string>(
        "arm_joint_names", std::string("joint1;joint2;joint3;joint4"),
        "Arm joint names, ';'-separated, in trajectory order"),
      BT::InputPort<std::string>(
        "joint_states_topic", std::string("/joint_states"),
        "JointState topic used for the already-carrying check"),
      BT::InputPort<double>("arm_move_time", 4.0, "Seconds allotted per arm move"),
      BT::InputPort<double>(
        "open_position", 0.02501155674647538, "Gripper open command [m]"),
      BT::InputPort<double>(
        "grasp_position", 0.002, "Gripper grasp command [m]"),
      BT::InputPort<double>(
        "max_effort", 0.0, "Gripper max effort (0 = controller default)"),
      BT::InputPort<double>(
        "grasp_settle_time", 1.0, "Seconds to wait after the grasp before lifting"),
      BT::InputPort<double>(
        "carry_tolerance", 0.12,
        "Max per-joint error [rad] to treat the arm as already at P_REAR_CARRY"),
      BT::InputPort<double>(
        "joint_state_timeout", 2.0,
        "Seconds to wait for /joint_states before running the full pick anyway"),
      BT::InputPort<double>(
        "server_timeout", 5.0, "Seconds to wait for the two action servers"),
      BT::InputPort<double>(
        "step_timeout", 15.0, "Per-step timeout [s] before the pick fails"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Kind { ARM, GRIPPER };
  enum class Phase { CHECK_CARRY, SEND, WAIT_ACCEPT, WAIT_RESULT, POST_DELAY };

  struct Step
  {
    Kind kind;
    std::array<double, 4> arm_pose;   // used when kind == ARM
    double gripper_pos;               // used when kind == GRIPPER
    double post_delay;                // seconds to hold after the step succeeds
    const char * label;
  };

  void buildSequence();
  void jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg);
  BT::NodeStatus fail(const char * why);
  void cancelActive();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr arm_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::string arm_action_name_;
  std::string gripper_action_name_;
  std::vector<std::string> arm_joint_names_;
  double arm_move_time_ {4.0};
  double open_position_ {0.02501155674647538};
  double grasp_position_ {0.002};
  double max_effort_ {0.0};
  double grasp_settle_time_ {1.0};
  double carry_tolerance_ {0.12};
  double joint_state_timeout_ {2.0};
  double server_timeout_ {5.0};
  double step_timeout_ {15.0};

  std::vector<double> current_pose_;   // arm joints in arm_joint_names_ order
  bool have_joint_state_ {false};

  std::vector<Step> steps_;
  std::size_t step_idx_ {0};
  Phase phase_ {Phase::CHECK_CARRY};
  bool servers_ready_ {false};

  rclcpp::Time start_time_;
  rclcpp::Time step_start_time_;
  rclcpp::Time delay_start_time_;

  std::shared_future<ArmGoalHandle::SharedPtr> arm_goal_future_;
  std::shared_future<GripGoalHandle::SharedPtr> grip_goal_future_;
  ArmGoalHandle::SharedPtr arm_goal_handle_;
  GripGoalHandle::SharedPtr grip_goal_handle_;
  std::shared_future<ArmGoalHandle::WrappedResult> arm_result_future_;
  std::shared_future<GripGoalHandle::WrappedResult> grip_result_future_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__REAR_PICK_OBJECT_ACTION_HPP_
