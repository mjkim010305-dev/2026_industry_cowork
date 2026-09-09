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

#ifndef CUSTOM_NAV2_BT_PLUGINS__LOCK_APPROACH_POSE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__LOCK_APPROACH_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Freeze ("lock") the obstacle pose once, so every downstream node works
 * from a single fixed estimate instead of chasing a live perception stream.
 *
 * This is the thin BT wrapper for the `LockApproachPose` step in
 * bt_perception_stabilizer_design.md: "copy the value to the blackboard at the
 * stable moment, then stop re-subscribing".
 *
 * Behaviour:
 *  - on entry the lock is cleared;
 *  - while unlocked it waits (RUNNING) for one fresh pose on `pose_topic`
 *    (age <= `pose_timeout`; if `require_orientation` it must also carry a
 *    non-identity quaternion), then latches it;
 *  - once latched it returns SUCCESS every tick, re-publishing the frozen pose
 *    on `locked_pose`, and never reads the topic again;
 *  - FAILURE if no acceptable pose arrives within `acquire_timeout`;
 *  - onHalted clears the lock, so re-entering the branch re-locks a fresh pose.
 *
 * `pose_topic` defaults to `/movable_obstacle/pose`; point it at
 * `/obstacle_pose_stable` once the Perception Stabilizer node (task 3) exists.
 */
class LockApproachPoseAction : public BT::StatefulActionNode
{
public:
  LockApproachPoseAction(const std::string & name, const BT::NodeConfiguration & conf);
  LockApproachPoseAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "pose_topic", std::string("/movable_obstacle/pose"),
        "geometry_msgs/PoseStamped topic with the obstacle pose (map frame)"),
      BT::InputPort<double>("pose_timeout", 2.0, "Max age [s] of a pose to accept for the lock"),
      BT::InputPort<double>(
        "acquire_timeout", 5.0,
        "Fail if no acceptable pose is latched within this many seconds"),
      BT::InputPort<bool>(
        "require_orientation", false,
        "Only lock a pose whose quaternion is a real (non-identity) orientation"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "locked_pose", "The frozen obstacle pose (written once, re-emitted every tick)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  static bool hasRealOrientation(const geometry_msgs::msg::Quaternion & q);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped last_pose_;
  rclcpp::Time last_pose_time_;
  bool pose_received_ {false};

  double pose_timeout_ {2.0};
  double acquire_timeout_ {5.0};
  bool require_orientation_ {false};

  bool locked_ {false};
  geometry_msgs::msg::PoseStamped locked_pose_;
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__LOCK_APPROACH_POSE_ACTION_HPP_
