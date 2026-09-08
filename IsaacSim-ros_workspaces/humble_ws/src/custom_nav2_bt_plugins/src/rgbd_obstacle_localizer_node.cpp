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

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "custom_nav2_bt_plugins/movable_obstacle_localizer.hpp"

namespace custom_nav2_bt_plugins
{

/**
 * @brief Host node for MovableObstacleLocalizer plugins.
 *
 * Consumes the RealSense depth stream + intrinsics, runs the configured
 * localizer plugin to get the obstacle centroid in the camera optical frame,
 * TF-transforms it into `target_frame` (map), and publishes it as a
 * PoseStamped that the behavior tree uses to build its approach sub-goal.
 *
 * If `gate_topic` is set (the VLM's is_movable boolean), the pose is published
 * only while that boolean is true and fresh - otherwise the VLM has not
 * flagged a movable obstacle and there is nothing to localize.
 */
class RgbdObstacleLocalizer : public rclcpp::Node
{
public:
  explicit RgbdObstacleLocalizer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("rgbd_obstacle_localizer", options),
    loader_("custom_nav2_bt_plugins", "custom_nav2_bt_plugins::MovableObstacleLocalizer")
  {
    depth_topic_ = declare_parameter(
      "depth_topic", std::string("/depth_camera"));
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", std::string("/front_camera_info"));
    gate_topic_ = declare_parameter("gate_topic", std::string("/obstacle/is_movable"));
    obstacle_pose_topic_ = declare_parameter(
      "obstacle_pose_topic", std::string("/movable_obstacle/pose"));
    target_frame_ = declare_parameter("target_frame", std::string("map"));
    publish_rate_ = declare_parameter("publish_rate", 10.0);
    data_timeout_ = declare_parameter("data_timeout", 1.0);
    gate_timeout_ = declare_parameter("gate_timeout", 2.0);
    transform_timeout_ = declare_parameter("transform_timeout", 0.2);
    // Use the latest available TF instead of the depth frame's stamp. The
    // obstacle is quasi-static while the robot approaches it, and this avoids
    // "extrapolation into the past" whenever the TF tree and this node briefly
    // disagree on time (e.g. right after startup, or on sim-time skew).
    use_latest_tf_ = declare_parameter("use_latest_tf", true);
    localizer_name_ = declare_parameter("localizer_name", std::string("depth_localizer"));
    const std::string plugin = declare_parameter(
      "localizer_plugin",
      std::string("custom_nav2_bt_plugins::DepthCentroidLocalizer"));

    localizer_ = loader_.createSharedInstance(plugin);
    localizer_->configure(this, localizer_name_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      obstacle_pose_topic_, rclcpp::QoS(10));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RgbdObstacleLocalizer::onDepth, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {last_info_ = msg;});
    if (!gate_topic_.empty()) {
      gate_sub_ = create_subscription<std_msgs::msg::Bool>(
        gate_topic_, rclcpp::QoS(10),
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          gate_ = msg->data;
          last_gate_time_ = now();
        });
    }

    const double hz = std::max(1.0, publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz)),
      std::bind(&RgbdObstacleLocalizer::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "rgbd_obstacle_localizer: depth='%s' info='%s' gate='%s' -> '%s' in '%s' @ %.1f Hz "
      "(localizer=%s)",
      depth_topic_.c_str(), camera_info_topic_.c_str(),
      gate_topic_.empty() ? "(none)" : gate_topic_.c_str(),
      obstacle_pose_topic_.c_str(), target_frame_.c_str(), hz, plugin.c_str());
  }

private:
  void onDepth(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    last_depth_ = msg;
    last_depth_time_ = now();
  }

  void onTimer()
  {
    // Gate: only localize while the VLM flags a movable obstacle.
    if (!gate_topic_.empty()) {
      const bool gate_fresh =
        last_gate_time_.nanoseconds() != 0 &&
        (now() - last_gate_time_).seconds() <= gate_timeout_;
      if (!gate_fresh || !gate_) {
        return;
      }
    }

    if (!last_depth_ || (now() - last_depth_time_).seconds() > data_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no fresh depth on '%s'", depth_topic_.c_str());
      return;
    }
    if (!last_info_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no CameraInfo on '%s' yet", camera_info_topic_.c_str());
      return;
    }

    const LocalizeResult r = localizer_->localize(last_depth_, last_info_);
    if (!r.ok) {
      RCLCPP_DEBUG(get_logger(), "localizer returned no obstacle this frame");
      return;
    }

    geometry_msgs::msg::PointStamped p_cam;
    p_cam.header = last_depth_->header;   // optical frame + frame time
    p_cam.point.x = r.x;
    p_cam.point.y = r.y;
    p_cam.point.z = r.z;

    geometry_msgs::msg::PointStamped p_map;
    try {
      if (use_latest_tf_) {
        const auto tf = tf_buffer_->lookupTransform(
          target_frame_, p_cam.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_));
        tf2::doTransform(p_cam, p_map, tf);
      } else {
        p_map = tf_buffer_->transform(
          p_cam, target_frame_, tf2::durationFromSec(transform_timeout_));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF %s <- %s failed: %s",
        target_frame_.c_str(), p_cam.header.frame_id.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = target_frame_;
    out.header.stamp = now();
    out.pose.position = p_map.point;
    out.pose.orientation.w = 1.0;   // heading is derived by the BT from the path
    pose_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "obstacle @ %s: (%.2f, %.2f, %.2f)  range=%.2f m  -> '%s'",
      target_frame_.c_str(), out.pose.position.x, out.pose.position.y,
      out.pose.position.z, r.distance, obstacle_pose_topic_.c_str());
  }

  pluginlib::ClassLoader<MovableObstacleLocalizer> loader_;
  MovableObstacleLocalizer::Ptr localizer_;

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string gate_topic_;
  std::string obstacle_pose_topic_;
  std::string target_frame_;
  std::string localizer_name_;
  double publish_rate_ {10.0};
  double data_timeout_ {1.0};
  double gate_timeout_ {2.0};
  double transform_timeout_ {0.2};
  bool use_latest_tf_ {true};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_info_;
  rclcpp::Time last_depth_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gate_time_ {0, 0, RCL_ROS_TIME};
  bool gate_ {false};
};

}  // namespace custom_nav2_bt_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<custom_nav2_bt_plugins::RgbdObstacleLocalizer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
