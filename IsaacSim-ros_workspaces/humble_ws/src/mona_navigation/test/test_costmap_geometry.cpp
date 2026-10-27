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

#include "mona_navigation/costmap_geometry.hpp"

using namespace mona_navigation::detail;  // NOLINT

namespace
{
// 100x100 cells of 0.05 m (5 m x 5 m), origin (-2.5,-2.5), all free.
CostGrid make_grid()
{
  CostGrid g;
  g.size_x = 100;
  g.size_y = 100;
  g.res = 0.05;
  g.ox = -2.5;
  g.oy = -2.5;
  g.data.assign(10000, 0);
  return g;
}

void set_box(CostGrid & g, double x0, double x1, double y0, double y1, uint8_t v)
{
  for (int cy = 0; cy < g.size_y; ++cy) {
    for (int cx = 0; cx < g.size_x; ++cx) {
      const double x = g.ox + (cx + 0.5) * g.res, y = g.oy + (cy + 0.5) * g.res;
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
        g.data[static_cast<size_t>(cy) * g.size_x + cx] = v;
      }
    }
  }
}
}  // namespace

TEST(CostmapGeometry, SideClearBothFree)
{
  const auto g = make_grid();
  EXPECT_EQ(side_clear(g, {0, 0, 0}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::LEFT);
}

TEST(CostmapGeometry, LeftBlockedGivesRight)
{
  auto g = make_grid();
  set_box(g, 0.0, 2.0, 0.5, 1.0, 254);   // wall on the left
  EXPECT_EQ(side_clear(g, {0, 0, 0}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::RIGHT);
}

TEST(CostmapGeometry, BothBlockedGivesNone)
{
  auto g = make_grid();
  set_box(g, 0.0, 2.0, 0.5, 1.0, 254);
  set_box(g, 0.0, 2.0, -1.0, -0.5, 254);
  EXPECT_EQ(side_clear(g, {0, 0, 0}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::NONE);
}

TEST(CostmapGeometry, RotatedRobot)
{
  auto g = make_grid();
  // robot at (0,0) facing +y (yaw 90 deg): its left is world -x, its right is world +x.
  set_box(g, -1.2, -0.4, 0.0, 2.0, 254);   // blocks robot-left
  EXPECT_EQ(side_clear(g, {0, 0, M_PI / 2}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::RIGHT);
  auto h = make_grid();
  set_box(h, 0.4, 1.2, 0.0, 2.0, 254);     // blocks robot-right
  EXPECT_EQ(side_clear(h, {0, 0, M_PI / 2}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::LEFT);
}

TEST(CostmapGeometry, OutOfBoundsAndUnknownAreNotClear)
{
  const auto g = make_grid();
  // robot near the +x edge: the windows lie mostly outside the grid
  EXPECT_EQ(side_clear(g, {2.2, 0, 0}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::NONE);
  auto u = make_grid();
  set_box(u, 0.0, 2.0, 0.3, 1.2, kCostUnknown);
  EXPECT_EQ(side_clear(u, {0, 0, 0}, 0.35, 0.05, 0.5, 0.2, 1.3, 254), Side::RIGHT);
}

TEST(CostmapGeometry, PathClear)
{
  auto g = make_grid();
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i <= 40; ++i) {pts.emplace_back(-2.0 + 0.1 * i, 0.0);}   // x from -2 to 2
  EXPECT_TRUE(path_clear_in_grid(g, pts, 3.0, 0.22, 254));
  set_box(g, 1.2, 1.3, 0.15, 0.25, 254);   // near path points that are > 3.0 m from the start
  EXPECT_TRUE(path_clear_in_grid(g, pts, 3.0, 0.22, 254));
  EXPECT_FALSE(path_clear_in_grid(g, pts, 3.5, 0.22, 254));
  set_box(g, -1.0, -0.9, 0.4, 0.5, 254);   // outside the radius
  EXPECT_TRUE(path_clear_in_grid(g, pts, 2.0, 0.22, 254));
  set_box(g, -1.0, -0.9, 0.1, 0.2, 254);   // inside the radius
  EXPECT_FALSE(path_clear_in_grid(g, pts, 2.0, 0.22, 254));
}

TEST(CostmapGeometry, StampGate)
{
  StampGate gate;
  EXPECT_TRUE(gate.accept(10, 500));
  EXPECT_FALSE(gate.accept(10, 500));
  EXPECT_FALSE(gate.accept(10, 400));
  EXPECT_TRUE(gate.accept(10, 600));
  EXPECT_TRUE(gate.accept(11, 0));
}
