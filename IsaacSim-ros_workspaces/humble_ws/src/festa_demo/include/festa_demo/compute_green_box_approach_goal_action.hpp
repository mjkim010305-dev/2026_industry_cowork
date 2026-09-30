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

#ifndef FESTA_DEMO__COMPUTE_GREEN_BOX_APPROACH_GOAL_ACTION_HPP_
#define FESTA_DEMO__COMPUTE_GREEN_BOX_APPROACH_GOAL_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "behaviortree_cpp_v3/action_node.h"

namespace festa_demo
{

/**
 * @brief Turn the detector's green-box pose (Lane P, `green_box/pose`) into a
 * standoff goal that Nav2's ComputePathToPose/FollowPath can drive to, so the
 * robot ends up squared up in front of the box instead of on top of it.
 *
 * goal = box_xy - standoff_distance * unit(robot -> box), yaw = atan2(box - robot)
 * i.e. the goal sits `standoff_distance` short of the box along the line from
 * the robot's current position to the box, facing the box. Robot pose comes
 * from TF (`global_frame` <- `robot_base_frame`), matching the pattern already
 * used by ApproachObstacleNormalAction/ComputeStraightPathToPoseAction in this
 * package (`nav2_util::getCurrentPose`).
 *
 * If the robot is already within `standoff_distance` of the box, the goal is
 * the robot's current pose and heading (hold position): a goal behind the
 * robot would need a slow in-place turn next to the box (g8). The final turn
 * to face the box is left to FinalApproachStop.
 *
 * FAILURE (with a log) if the last `pose_topic` message is missing/stale
 * (age > `max_age`) or the TF lookup fails - either way there is nothing
 * usable to compute a goal from.
 */
class ComputeGreenBoxApproachGoalAction : public BT::SyncActionNode
{
public:
  ComputeGreenBoxApproachGoalAction(const std::string & name, const BT::NodeConfiguration & conf);
  ComputeGreenBoxApproachGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "pose_topic", std::string("green_box/pose"),
        "geometry_msgs/PoseStamped published by the green-box detector"),
      BT::InputPort<std::string>(
        "global_frame", std::string("map"),
        "Frame the goal is expressed in - must match the detector's output_frame"),
      BT::InputPort<std::string>(
        "robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>(
        "standoff_distance", 0.8, "Stop this far [m] short of the box centre, facing it. 0.8, not 0.5: at 0.5 m from the centre only ~0.15 m is left between a 0.3 m box and the robot, and with the goal checker's 0.35 m xy tolerance FollowPath ended touching the box, so the final turn pushed it (g7)"),
      BT::InputPort<double>(
        "max_age", 1.0, "Max age [s] of the last pose_topic message before it counts as stale"),
      BT::InputPort<double>("transform_tolerance", 0.1, "TF tolerance [s]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "approach_goal", "Standoff goal in front of the box, for ComputePathToPose"),
    };
  }

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  bool pose_received_ {false};
};

}  // namespace festa_demo

#endif  // FESTA_DEMO__COMPUTE_GREEN_BOX_APPROACH_GOAL_ACTION_HPP_
