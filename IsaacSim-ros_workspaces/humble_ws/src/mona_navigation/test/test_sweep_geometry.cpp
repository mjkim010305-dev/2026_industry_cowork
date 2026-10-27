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

#include <gtest/gtest.h>

#include <cmath>

#include "mona_navigation/compute_sweep_poses_action.hpp"
#include "mona_navigation/creep_to_face_action.hpp"

using namespace mona_navigation::detail;  // NOLINT

namespace
{
mona_interfaces::msg::PathObstacle obstacle(double nx, double ny)
{
  mona_interfaces::msg::PathObstacle o;
  o.present = true;
  o.face_start.x = 2.0; o.face_start.y = -0.5;
  o.face_end.x = 2.0; o.face_end.y = 0.5;
  o.normal.x = nx; o.normal.y = ny;
  return o;
}
}  // namespace

TEST(SweepGeometry, PosesFacingObject)
{
  // Object face at x=2, normal toward robot = -x, robot approaches along +x.
  const auto p = compute_sweep_poses(obstacle(-1.0, 0.0), 1.1, 0.15);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->staging_x, 0.9, 1e-9);
  EXPECT_NEAR(p->face_x, 1.85, 1e-9);
  EXPECT_NEAR(p->staging_y, 0.0, 1e-9);
  EXPECT_NEAR(p->yaw, 0.0, 1e-9);
  EXPECT_NEAR(p->mid_x, 2.0, 1e-9);  // look_pose position = face mid
  EXPECT_NEAR(p->mid_y, 0.0, 1e-9);
}

TEST(SweepGeometry, PoseYawAndNormalization)
{
  const auto p = compute_sweep_poses(obstacle(0.0, 2.0), 1.0, 0.1);  // not unit
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->staging_y, 1.0, 1e-9);
  EXPECT_NEAR(p->yaw, -M_PI / 2, 1e-9);
  const auto ps = make_pose(1.0, 2.0, p->yaw, "map");
  EXPECT_NEAR(yaw_from_quat(ps.pose.orientation), -M_PI / 2, 1e-9);
}

TEST(SweepGeometry, ZeroNormalRejected)
{
  EXPECT_FALSE(compute_sweep_poses(obstacle(0.0, 0.0), 1.1, 0.15).has_value());
}

TEST(SweepGeometry, AlongAndLateral)
{
  // target (1,0) heading +x
  EXPECT_NEAR(along_remaining(0.0, 0.0, 1.0, 0.0, 0.0), 1.0, 1e-9);
  EXPECT_LT(along_remaining(1.2, 0.0, 1.0, 0.0, 0.0), 0.0);
  EXPECT_NEAR(lateral_error(0.0, 0.3, 1.0, 0.0, 0.0), 0.3, 1e-9);   // left of line
  EXPECT_NEAR(lateral_error(0.0, -0.3, 1.0, 0.0, 0.0), -0.3, 1e-9);
}

TEST(SweepGeometry, ControlSigns)
{
  EXPECT_GT(rotate_cmd(0.0, 1.0), 0.0);
  EXPECT_LE(std::fabs(rotate_cmd(0.0, 3.0)), kMaxAngular);
  EXPECT_GE(std::fabs(rotate_cmd(0.0, 0.06)), kMinRotateAngular);
  EXPECT_LT(drive_cmd(0.0, 0.0, 0.3), 0.0);  // left of line -> turn right
  EXPECT_GT(drive_cmd(0.0, 0.0, -0.3), 0.0);
  EXPECT_NEAR(wrap_angle(3 * M_PI), M_PI, 1e-9);
}

TEST(SweepGeometry, StallTracker)
{
  StallTracker s;
  EXPECT_FALSE(s.update(0.0, 0.05, 0.0, 0.3, 2.0));
  EXPECT_FALSE(s.update(1.9, 0.05, 0.001, 0.3, 2.0));
  EXPECT_TRUE(s.update(2.1, 0.05, 0.001, 0.3, 2.0));
  EXPECT_FALSE(s.update(2.2, 0.05, 0.05, 0.3, 2.0));   // moving again resets
  EXPECT_FALSE(s.update(4.5, 0.02, 0.0, 0.3, 2.0));    // cmd below min: not judged
  EXPECT_FALSE(s.update(5.0, 0.05, 0.0, 0.3, 2.0));
  EXPECT_FALSE(s.update(6.0, 0.05, 0.0, 0.3, 2.0));
  EXPECT_TRUE(s.update(7.1, 0.05, 0.0, 0.3, 2.0));
}
