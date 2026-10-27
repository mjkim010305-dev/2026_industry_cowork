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

#ifndef MONA_NAVIGATION__COSTMAP_GEOMETRY_HPP_
#define MONA_NAVIGATION__COSTMAP_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace mona_navigation
{
namespace detail
{

constexpr uint8_t kCostUnknown = 255;

/// Plain copy of a nav2_msgs/Costmap (axis-aligned; origin = lower-left corner of cell (0,0)).
struct CostGrid
{
  int size_x {0}, size_y {0};
  double res {0.05};
  double ox {0.0}, oy {0.0};
  std::vector<uint8_t> data;   // row-major, index = y * size_x + x

  bool valid() const
  {
    return size_x > 0 && size_y > 0 && res > 0.0 &&
           data.size() == static_cast<size_t>(size_x) * static_cast<size_t>(size_y);
  }

  /// Cell index of a world point, false if outside the grid.
  bool cell(double wx, double wy, int & cx, int & cy) const
  {
    const double fx = std::floor((wx - ox) / res);
    const double fy = std::floor((wy - oy) / res);
    if (fx < 0.0 || fy < 0.0 || fx >= size_x || fy >= size_y) {return false;}
    cx = static_cast<int>(fx);
    cy = static_cast<int>(fy);
    return true;
  }

  uint8_t at(int cx, int cy) const {return data[static_cast<size_t>(cy) * size_x + cx];}
};

/// Pose of the robot base in the grid frame.
struct Pose2D
{
  double x {0.0}, y {0.0}, yaw {0.0};
};

inline bool is_lethal(uint8_t v, uint8_t lethal)
{
  return v != kCostUnknown && v >= lethal;
}

/**
 * Window [x0,x1] x [y0,y1] given in the robot base frame (x forward, y left), sampled every
 * grid resolution. Clear iff no sample hits a lethal cell and >= min_known of the samples lie
 * inside the grid on a known (not 255) cell.
 */
inline bool window_clear(
  const CostGrid & g, const Pose2D & robot, double x0, double x1, double y0, double y1,
  uint8_t lethal, double min_known = 0.8)
{
  if (!g.valid() || x1 <= x0 || y1 <= y0) {return false;}
  const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
  const int nx = std::max(1, static_cast<int>(std::ceil((x1 - x0) / g.res)));
  const int ny = std::max(1, static_cast<int>(std::ceil((y1 - y0) / g.res)));
  int total = 0, known = 0;
  for (int i = 0; i < nx; ++i) {
    const double lx = x0 + (i + 0.5) * (x1 - x0) / nx;
    for (int j = 0; j < ny; ++j) {
      const double ly = y0 + (j + 0.5) * (y1 - y0) / ny;
      ++total;
      int cx, cy;
      if (!g.cell(robot.x + c * lx - s * ly, robot.y + s * lx + c * ly, cx, cy)) {continue;}
      const uint8_t v = g.at(cx, cy);
      if (v == kCostUnknown) {continue;}
      ++known;
      if (is_lethal(v, lethal)) {return false;}
    }
  }
  return total > 0 && known >= min_known * total;
}

enum class Side { NONE, LEFT, RIGHT };

/// Left window is tested first; if both are clear the result is LEFT.
inline Side side_clear(
  const CostGrid & g, const Pose2D & robot, double half, double margin, double width,
  double along_min, double along_len, uint8_t lethal)
{
  const double y_in = half + margin;
  const double x0 = along_min, x1 = along_min + along_len;
  if (window_clear(g, robot, x0, x1, y_in, y_in + width, lethal)) {return Side::LEFT;}
  if (window_clear(g, robot, x0, x1, -(y_in + width), -y_in, lethal)) {return Side::RIGHT;}
  return Side::NONE;
}

/**
 * True iff no lethal cell lies within `radius` of any path point in the first check_dist metres
 * (measured along the polyline from its first point). Points are in the grid frame; cells
 * outside the grid are ignored.
 */
inline bool path_clear_in_grid(
  const CostGrid & g, const std::vector<std::pair<double, double>> & pts,
  double check_dist, double radius, uint8_t lethal)
{
  if (!g.valid()) {return false;}
  double travelled = 0.0;
  for (size_t k = 0; k < pts.size(); ++k) {
    if (k > 0) {
      travelled += std::hypot(pts[k].first - pts[k - 1].first, pts[k].second - pts[k - 1].second);
      if (travelled > check_dist) {break;}
    }
    const double px = pts[k].first, py = pts[k].second;
    const int x_lo = static_cast<int>(std::floor((px - radius - g.ox) / g.res));
    const int x_hi = static_cast<int>(std::floor((px + radius - g.ox) / g.res));
    const int y_lo = static_cast<int>(std::floor((py - radius - g.oy) / g.res));
    const int y_hi = static_cast<int>(std::floor((py + radius - g.oy) / g.res));
    for (int cy = std::max(0, y_lo); cy <= std::min(g.size_y - 1, y_hi); ++cy) {
      for (int cx = std::max(0, x_lo); cx <= std::min(g.size_x - 1, x_hi); ++cx) {
        if (!is_lethal(g.at(cx, cy), lethal)) {continue;}
        const double dx = g.ox + (cx + 0.5) * g.res - px;
        const double dy = g.oy + (cy + 0.5) * g.res - py;
        if (std::hypot(dx, dy) <= radius) {return false;}
      }
    }
  }
  return true;
}

/// Accepts a message stamp only if it is newer than the last accepted one.
struct StampGate
{
  bool has {false};
  int64_t last_ns {0};

  bool accept(int32_t sec, uint32_t nanosec)
  {
    const int64_t ns = static_cast<int64_t>(sec) * 1000000000LL + nanosec;
    if (has && ns <= last_ns) {return false;}
    has = true;
    last_ns = ns;
    return true;
  }
};

}  // namespace detail
}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__COSTMAP_GEOMETRY_HPP_
