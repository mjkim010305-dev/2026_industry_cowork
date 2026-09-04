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

#ifndef CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_LOCALIZER_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_LOCALIZER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace custom_nav2_bt_plugins
{

/// @brief 3D position of the blocking obstacle in the depth camera's optical frame.
struct LocalizeResult
{
  bool ok {false};       ///< true when a usable obstacle position was extracted
  double x {0.0};         ///< [m] right   (optical frame)
  double y {0.0};         ///< [m] down    (optical frame)
  double z {0.0};         ///< [m] forward (optical frame)
  double distance {-1.0}; ///< [m] range to the obstacle centroid, < 0 if unknown
};

/**
 * @brief pluginlib base class: extract the obstacle's 3D position from one
 * RGB-D frame.
 *
 * The VLM decides *whether* there is a movable obstacle (and whether it is
 * still there) and publishes those as plain booleans. This plugin's only job
 * is the geometry: given the RealSense depth image + intrinsics, return the
 * obstacle centroid as a point in the depth optical frame. The host node
 * (`rgbd_obstacle_localizer`) then TF-transforms that point into the map
 * frame and publishes it as a PoseStamped for the behavior tree.
 *
 * Swap in a smarter implementation (e.g. one that also consumes a VLM
 * bounding box) via the host node's `localizer_plugin` parameter.
 */
class MovableObstacleLocalizer
{
public:
  using Ptr = std::shared_ptr<MovableObstacleLocalizer>;

  virtual ~MovableObstacleLocalizer() = default;

  /**
   * @brief One-time setup. Parameters are declared under the "<name>." prefix.
   * @param node non-owning back-pointer to the host node (outlives the plugin)
   * @param name plugin instance name, used as the parameter namespace
   */
  virtual void configure(rclcpp::Node * node, const std::string & name) = 0;

  /**
   * @brief Locate the obstacle in one depth frame.
   * @param depth       depth image, 16UC1/mono16 (mm) or 32FC1 (m); never null
   * @param camera_info matching CameraInfo (intrinsics); never null
   */
  virtual LocalizeResult localize(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) = 0;

  /// @brief Human-readable instance name for logs.
  virtual std::string getName() const = 0;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_LOCALIZER_HPP_
