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

#include "mona_navigation/push_geometry.hpp"

using namespace mona_navigation::detail;  // NOLINT

namespace
{
nav_msgs::msg::Path line(size_t n, double step)
{
  nav_msgs::msg::Path p;
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = step * static_cast<double>(i);
    p.poses.push_back(ps);
  }
  return p;
}
}  // namespace

TEST(PushGeometry, PathLength)
{
  EXPECT_DOUBLE_EQ(path_length(line(0, 1.0)), 0.0);
  EXPECT_DOUBLE_EQ(path_length(line(1, 1.0)), 0.0);
  EXPECT_DOUBLE_EQ(path_length(line(5, 0.5)), 2.0);
}

TEST(PushGeometry, PathLengthWithinRule)
{
  const auto ref = line(11, 1.0);  // 10 m -> limit 1.5*10+3 = 18
  EXPECT_TRUE(path_length_within(line(19, 1.0), ref, 1.5, 3.0));    // 18 (boundary)
  EXPECT_FALSE(path_length_within(line(20, 1.0), ref, 1.5, 3.0));   // 19
  EXPECT_FALSE(path_length_within(line(1, 1.0), ref, 1.5, 3.0));    // < 2 poses
  EXPECT_FALSE(path_length_within(line(0, 1.0), line(0, 1.0), 1.5, 3.0));
  EXPECT_TRUE(path_length_within(line(2, 1.0), line(0, 1.0), 1.5, 3.0));  // empty ref: slack only
}

TEST(PushGeometry, TravelAccumulates)
{
  TravelTracker t;
  t.update(5.0, 5.0);  // first sample is the origin
  EXPECT_DOUBLE_EQ(t.total, 0.0);
  t.update(5.3, 5.4);
  t.update(5.3, 5.4);
  t.update(5.3, 6.4);
  EXPECT_NEAR(t.total, 1.5, 1e-12);
  t.reset();
  EXPECT_DOUBLE_EQ(t.total, 0.0);
  t.update(0.0, 0.0);
  t.update(0.1, 0.0);
  EXPECT_NEAR(t.total, 0.1, 1e-12);
}

TEST(PushGeometry, SplitSemicolon)
{
  const auto v = split_semicolon("joint1; joint2;joint3;");
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[1], "joint2");
}

TEST(PushGeometry, TwoPointTrajectory)
{
  const auto t = two_point_trajectory({"a", "b"}, {0.1, 0.2}, {1.0, 2.0}, 2.5);
  ASSERT_EQ(t.points.size(), 2u);
  EXPECT_EQ(t.joint_names.size(), 2u);
  EXPECT_EQ(t.points[0].positions[1], 0.2);
  EXPECT_EQ(t.points[0].time_from_start.sec, 0);
  EXPECT_EQ(t.points[1].positions[0], 1.0);
  EXPECT_EQ(t.points[1].time_from_start.sec, 2);
  EXPECT_EQ(t.points[1].time_from_start.nanosec, 500000000u);
}
