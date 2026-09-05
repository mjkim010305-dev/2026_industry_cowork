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

#ifndef CUSTOM_NAV2_BT_PLUGINS__DEPTH_THRESHOLD_DETECTOR_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__DEPTH_THRESHOLD_DETECTOR_HPP_

#include <string>

#include "custom_nav2_bt_plugins/movable_obstacle_detector.hpp"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Depth-only movable-obstacle detector.
 *
 * Looks at a central rectangular ROI of the depth image. If the fraction of
 * valid ROI pixels whose range is below `stop_distance` exceeds
 * `min_fill_ratio`, an obstacle is reported "present". If that condition holds
 * continuously for `movable_confirm_time` seconds, the obstacle is also
 * reported "movable": in this navigation scenario the global planner has
 * already failed to route around the blocker, so a stable close object is
 * treated as something the robot should drive up to and wait out. The movable
 * latch is released once the ROI has stayed clear for `clear_time` seconds.
 *
 * This is deliberately simple and swappable — implement a smarter
 * MovableObstacleDetector (semantic segmentation, tracking, ...) and select it
 * with the host node's `detector_plugin` parameter.
 */
class DepthThresholdDetector : public MovableObstacleDetector
{
public:
  void configure(rclcpp::Node * node, const std::string & name) override;

  ObstacleReading detect(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override;

  std::string getName() const override {return name_;}

private:
  rclcpp::Node * node_ {nullptr};
  std::string name_ {"depth_detector"};

  // Parameters (see class docs).
  double roi_width_ratio_ {0.4};
  double roi_height_ratio_ {0.6};
  double stop_distance_ {0.8};
  double min_range_ {0.15};
  double min_fill_ratio_ {0.06};
  double movable_confirm_time_ {1.5};
  double clear_time_ {1.0};

  // Temporal state for the movable latch.
  rclcpp::Time present_since_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time clear_since_ {0, 0, RCL_ROS_TIME};
  bool movable_latched_ {false};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__DEPTH_THRESHOLD_DETECTOR_HPP_
