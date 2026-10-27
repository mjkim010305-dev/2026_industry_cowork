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

#ifndef MONA_NAVIGATION__PUSH_GEOMETRY_HPP_
#define MONA_NAVIGATION__PUSH_GEOMETRY_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace mona_navigation
{
namespace detail
{

/// Polyline length of a path [m] (0 for fewer than 2 poses).
inline double path_length(const nav_msgs::msg::Path & p)
{
  double len = 0.0;
  for (size_t i = 1; i < p.poses.size(); ++i) {
    const auto & a = p.poses[i - 1].pose.position;
    const auto & b = p.poses[i].pose.position;
    len += std::hypot(b.x - a.x, b.y - a.y);
  }
  return len;
}

/// Candidate is acceptable iff it has >= 2 poses and len <= ratio * len(reference) + slack.
inline bool path_length_within(
  const nav_msgs::msg::Path & path, const nav_msgs::msg::Path & reference,
  double ratio, double slack)
{
  return path.poses.size() >= 2 && path_length(path) <= ratio * path_length(reference) + slack;
}

/// Distance travelled, accumulated from successive odom positions.
struct TravelTracker
{
  bool has_last {false};
  double x {0.0}, y {0.0};
  double total {0.0};

  void reset() {has_last = false; total = 0.0;}

  void update(double nx, double ny)
  {
    if (has_last) {
      total += std::hypot(nx - x, ny - y);
    }
    x = nx;
    y = ny;
    has_last = true;
  }
};

/// "a;b;c" -> {a, b, c}; empty items are dropped.
inline std::vector<std::string> split_semicolon(const std::string & s)
{
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= s.size()) {
    size_t end = s.find(';', start);
    if (end == std::string::npos) {end = s.size();}
    std::string item = s.substr(start, end - start);
    const auto b = item.find_first_not_of(" \t");
    const auto e = item.find_last_not_of(" \t");
    if (b != std::string::npos) {out.push_back(item.substr(b, e - b + 1));}
    start = end + 1;
  }
  return out;
}

/// Two-point trajectory: current at t=0, target at move_time (the Isaac bridge snaps a single
/// point in one step, so the start point is needed for it to interpolate).
inline trajectory_msgs::msg::JointTrajectory two_point_trajectory(
  const std::vector<std::string> & names, const std::vector<double> & current,
  const std::vector<double> & target, double move_time)
{
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = names;
  trajectory_msgs::msg::JointTrajectoryPoint p0, p1;
  p0.positions = current;
  p1.positions = target;
  p0.time_from_start.sec = 0;
  p0.time_from_start.nanosec = 0;
  const double whole = std::floor(move_time);
  p1.time_from_start.sec = static_cast<int32_t>(whole);
  p1.time_from_start.nanosec = static_cast<uint32_t>((move_time - whole) * 1e9);
  t.points = {p0, p1};
  return t;
}

}  // namespace detail
}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__PUSH_GEOMETRY_HPP_
