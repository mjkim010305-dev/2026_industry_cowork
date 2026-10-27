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

#ifndef MONA_NAVIGATION__COMPUTE_SWEEP_POSES_ACTION_HPP_
#define MONA_NAVIGATION__COMPUTE_SWEEP_POSES_ACTION_HPP_

#include <cmath>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mona_interfaces/msg/path_obstacle.hpp"

namespace mona_navigation
{
namespace detail
{

struct SweepPoses
{
  double staging_x, staging_y, face_x, face_y, yaw, mid_x, mid_y;
};

/// Staging/face poses from a PathObstacle face. nullopt if the normal is ~zero.
/// heading = atan2(-n.y, -n.x): the robot looks at the object. n points toward the robot.
inline std::optional<SweepPoses> compute_sweep_poses(
  const mona_interfaces::msg::PathObstacle & o, double staging_dist, double face_dist)
{
  const double nx = o.normal.x, ny = o.normal.y;
  const double len = std::hypot(nx, ny);
  if (len < 1e-3) {
    return std::nullopt;
  }
  const double ux = nx / len, uy = ny / len;
  const double mx = 0.5 * (o.face_start.x + o.face_end.x);
  const double my = 0.5 * (o.face_start.y + o.face_end.y);
  return SweepPoses{
    mx + ux * staging_dist, my + uy * staging_dist,
    mx + ux * face_dist, my + uy * face_dist,
    std::atan2(-uy, -ux), mx, my};
}

inline geometry_msgs::msg::PoseStamped make_pose(
  double x, double y, double yaw, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(0.5 * yaw);
  p.pose.orientation.w = std::cos(0.5 * yaw);
  return p;
}

}  // namespace detail

/**
 * @brief From a PathObstacle compute the nav goal in front of the face (staging_pose) and
 * the point to creep to (face_pose), both facing the object, in frame "map".
 * FAILURE if the obstacle is missing / not present / normal ~zero.
 */
class ComputeSweepPosesAction : public BT::SyncActionNode
{
public:
  ComputeSweepPosesAction(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(name, conf) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<mona_interfaces::msg::PathObstacle>("obstacle", "Obstacle to sweep"),
      BT::InputPort<double>("staging_dist", 1.1, "Staging distance from the face [m]"),
      BT::InputPort<double>("face_dist", 0.15, "Creep target distance from the face [m]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("staging_pose", "Pose to navigate to"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("face_pose", "Pose to creep to"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "look_pose", "Face mid, heading from robot to object (for CreepToFace rotate_only)"),
    };
  }
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__COMPUTE_SWEEP_POSES_ACTION_HPP_
