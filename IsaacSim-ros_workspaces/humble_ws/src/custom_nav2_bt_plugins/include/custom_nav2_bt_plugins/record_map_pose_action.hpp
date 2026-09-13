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
//
// NOTE: not currently wired into CMakeLists.txt/demo_nav2_params.yaml/
// demo.xml - the session reverted from /amcl_pose back to /odom
// (RecordOdomPoseAction) for Leg 2's record/return, keeping this file on
// disk rather than deleting it in case /amcl_pose is worth retrying later.

#ifndef CUSTOM_NAV2_BT_PLUGINS__RECORD_MAP_POSE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__RECORD_MAP_POSE_ACTION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Snapshot the robot's current pose (AMCL's /amcl_pose, i.e. the
 * map frame) onto the blackboard.
 *
 * Originally this read raw /odom instead (pure wheel-encoder dead
 * reckoning, no map/costmap involved at all - immune to the "goal
 * coincides with a just-placed object" collision-check problem that
 * map-frame navigation and DriveOnHeading/BackUp's lookahead kept hitting
 * near spot A). That worked for avoiding costmap collision checks, but
 * wheel odometry has its own failure mode: pushing against the obstacle's
 * resistance makes the wheels slip, and if the slip isn't symmetric
 * between the two sides, odometry integrates a yaw drift that never
 * physically happened - so the "recorded" pose itself could already be
 * wrong by the time DriveToRecordedPose reads it back. AMCL corrects
 * against the environment via laser scan matching, so wheel slip alone
 * doesn't fool it (as long as it's well converged) - the tradeoff being
 * /amcl_pose updates less often than /odom (only on qualifying motion
 * per update_min_d/update_min_a), which is fine for a one-shot snapshot
 * like this.
 *
 * Meant to be called right before an OdomLimitedDrive push, so
 * DriveToRecordedPose can later return to exactly this pose regardless of
 * how far the push actually traveled.
 *
 * StatefulActionNode: RUNNING until at least one /amcl_pose message has
 * been received (bounded by timeout), then SUCCESS.
 */
class RecordMapPoseAction : public BT::StatefulActionNode
{
public:
  RecordMapPoseAction(const std::string & name, const BT::NodeConfiguration & conf);
  RecordMapPoseAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "pose_topic", std::string("/amcl_pose"), "PoseWithCovarianceStamped topic"),
      BT::InputPort<double>("timeout", 3.0, "Seconds to wait for a pose message"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "The recorded pose"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  std::string pose_topic_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_pose_;
  rclcpp::Time start_time_;
  double timeout_ {3.0};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__RECORD_MAP_POSE_ACTION_HPP_
