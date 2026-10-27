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

#include "mona_navigation/compute_sweep_poses_action.hpp"

namespace mona_navigation
{

BT::NodeStatus ComputeSweepPosesAction::tick()
{
  mona_interfaces::msg::PathObstacle obstacle;
  if (!getInput("obstacle", obstacle) || !obstacle.present) {
    return BT::NodeStatus::FAILURE;
  }
  double staging_dist = 1.1, face_dist = 0.15;
  getInput("staging_dist", staging_dist);
  getInput("face_dist", face_dist);

  const auto poses = detail::compute_sweep_poses(obstacle, staging_dist, face_dist);
  if (!poses) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("staging_pose",
    detail::make_pose(poses->staging_x, poses->staging_y, poses->yaw, "map"));
  setOutput("face_pose", detail::make_pose(poses->face_x, poses->face_y, poses->yaw, "map"));
  setOutput("look_pose", detail::make_pose(poses->mid_x, poses->mid_y, poses->yaw, "map"));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::ComputeSweepPosesAction>("ComputeSweepPoses");
}
