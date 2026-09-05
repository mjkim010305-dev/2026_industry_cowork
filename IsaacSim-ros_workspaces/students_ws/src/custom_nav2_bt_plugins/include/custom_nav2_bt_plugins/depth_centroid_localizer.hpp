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

#ifndef CUSTOM_NAV2_BT_PLUGINS__DEPTH_CENTROID_LOCALIZER_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__DEPTH_CENTROID_LOCALIZER_HPP_

#include <string>

#include "custom_nav2_bt_plugins/movable_obstacle_localizer.hpp"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Depth-only obstacle localizer.
 *
 * Assumes the blocking obstacle is the nearest large thing in front of the
 * camera (it is on the path, dead ahead). Steps:
 *   1. scan a central ROI of the depth image, keep pixels in
 *      [min_range, max_range];
 *   2. take the `near_percentile` depth as a robust "nearest" value and keep
 *      every ROI pixel within `depth_band` of it;
 *   3. average those pixels -> centroid (u, v, depth);
 *   4. deproject with the CameraInfo intrinsics -> (x, y, z) in the optical
 *      frame.
 * Fails (ok = false) when fewer than `min_points` pixels survive.
 */
class DepthCentroidLocalizer : public MovableObstacleLocalizer
{
public:
  void configure(rclcpp::Node * node, const std::string & name) override;

  LocalizeResult localize(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) override;

  std::string getName() const override {return name_;}

private:
  rclcpp::Node * node_ {nullptr};
  std::string name_ {"depth_localizer"};

  double roi_width_ratio_ {0.4};
  double roi_height_ratio_ {0.6};
  double min_range_ {0.2};
  double max_range_ {6.0};
  double near_percentile_ {0.15};
  double depth_band_ {0.25};
  int min_points_ {50};
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__DEPTH_CENTROID_LOCALIZER_HPP_
