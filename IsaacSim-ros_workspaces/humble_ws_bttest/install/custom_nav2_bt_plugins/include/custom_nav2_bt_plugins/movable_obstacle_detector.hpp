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

#ifndef CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_DETECTOR_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_DETECTOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace custom_nav2_bt_plugins
{

/// @brief The two boolean facts produced from one RGB-D frame, plus a debug range.
struct ObstacleReading
{
  bool obstacle_present {false};    ///< something solid is right in front of the robot
  bool is_movable {false};          ///< that something is classified as a movable obstacle
  float nearest_distance {-1.0f};   ///< [m] closest range seen in the ROI, < 0 if unknown
};

/**
 * @brief pluginlib base class: turns an RGB-D frame into ObstacleReading.
 *
 * Concrete detectors (depth-threshold, learned segmentation, ...) implement
 * detect(). All ROS plumbing (subscriptions, publishers, timers) lives in the
 * host node `rgbd_obstacle_monitor`; a detector only receives the frames and a
 * back-pointer to the node for parameters / logging / clock.
 */
class MovableObstacleDetector
{
public:
  using Ptr = std::shared_ptr<MovableObstacleDetector>;

  virtual ~MovableObstacleDetector() = default;

  /**
   * @brief One-time setup. Parameters are declared under the "<name>." prefix.
   * @param node non-owning back-pointer to the host node (outlives the detector)
   * @param name plugin instance name, used as the parameter namespace
   */
  virtual void configure(rclcpp::Node * node, const std::string & name) = 0;

  /**
   * @brief Evaluate one depth frame.
   * @param depth       depth image, 16UC1/mono16 (mm) or 32FC1 (m); never null
   * @param camera_info matching CameraInfo, may be null when not subscribed
   */
  virtual ObstacleReading detect(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info) = 0;

  /// @brief Human-readable instance name for logs.
  virtual std::string getName() const = 0;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__MOVABLE_OBSTACLE_DETECTOR_HPP_
