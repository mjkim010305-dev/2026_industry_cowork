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

#ifndef MONA_NAVIGATION__MOVABLE_OBSTACLE_LAYER_HPP_
#define MONA_NAVIGATION__MOVABLE_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mona_navigation
{

/**
 * Costmap layer fed by movable_obstacle_map_node (/movable_obstacles/list).
 *
 * The lidar already puts every obstacle into the costmap as LETHAL; what it
 * cannot say is which ones the robot may push. This layer re-marks the
 * judged objects by verdict: fixed ones stay LETHAL, pushable ones get
 * `movable_cost`. The default keeps pushable objects impassable too
 * (INSCRIBED), so nothing changes for the planner until someone decides,
 * per deployment, that the push branch may be planned through.
 */
class MovableObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  MovableObstacleLayer() = default;
  ~MovableObstacleLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  struct Obj
  {
    double x, y, side;
    bool movable, pushable;
  };

  void onList(const std_msgs::msg::String::SharedPtr msg);
  static bool parse(const std::string & json, std::vector<Obj> & out, std::string & frame);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  // Owned here, not a temporary: RCLCPP_*_THROTTLE keeps a reference to the
  // clock it is given and calls now() on it later. A `*Clock::make_shared()`
  // argument segfaulted the whole Nav2 container (2026-09-23, core dump).
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MovableObstacleLayer")};
  std::mutex mutex_;
  std::vector<Obj> objects_;
  std::vector<Obj> last_drawn_;
  std::string list_topic_;
  std::string list_frame_;
  unsigned char movable_cost_{0};
  unsigned char fixed_cost_{0};
  double padding_{0.0};
  bool mark_fixed_{true};
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__MOVABLE_OBSTACLE_LAYER_HPP_
