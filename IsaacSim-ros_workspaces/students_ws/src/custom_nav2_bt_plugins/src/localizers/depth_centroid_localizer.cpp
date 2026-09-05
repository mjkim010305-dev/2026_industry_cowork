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

#include "custom_nav2_bt_plugins/depth_centroid_localizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace custom_nav2_bt_plugins
{

void DepthCentroidLocalizer::configure(rclcpp::Node * node, const std::string & name)
{
  node_ = node;
  name_ = name;

  auto pd = [&](const std::string & key, double def) {
      return node_->declare_parameter(name_ + "." + key, def);
    };
  roi_width_ratio_ = pd("roi_width_ratio", roi_width_ratio_);
  roi_height_ratio_ = pd("roi_height_ratio", roi_height_ratio_);
  min_range_ = pd("min_range", min_range_);
  max_range_ = pd("max_range", max_range_);
  near_percentile_ = pd("near_percentile", near_percentile_);
  depth_band_ = pd("depth_band", depth_band_);
  min_points_ = static_cast<int>(node_->declare_parameter(name_ + ".min_points", min_points_));

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] DepthCentroidLocalizer: roi=%.2fx%.2f, range=[%.2f, %.2f] m, "
    "near_percentile=%.2f, depth_band=%.2f m, min_points=%d",
    name_.c_str(), roi_width_ratio_, roi_height_ratio_, min_range_, max_range_,
    near_percentile_, depth_band_, min_points_);
}

LocalizeResult DepthCentroidLocalizer::localize(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  LocalizeResult out;

  const auto & img = *depth;
  const bool is_u16 =
    (img.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    img.encoding == sensor_msgs::image_encodings::MONO16);
  const bool is_f32 = (img.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  if ((!is_u16 && !is_f32) || img.width == 0 || img.height == 0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] unusable depth frame (encoding='%s', %ux%u)",
      name_.c_str(), img.encoding.c_str(), img.width, img.height);
    return out;
  }

  // Intrinsics from CameraInfo K = [fx 0 cx; 0 fy cy; 0 0 1].
  const double fx = camera_info->k[0];
  const double fy = camera_info->k[4];
  const double cx = camera_info->k[2];
  const double cy = camera_info->k[5];
  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "[%s] CameraInfo intrinsics look invalid (fx=%.1f fy=%.1f)", name_.c_str(), fx, fy);
    return out;
  }

  const int w = static_cast<int>(img.width);
  const int h = static_cast<int>(img.height);
  const int pixel_bytes = is_u16 ? 2 : 4;

  const int roi_w = std::clamp(static_cast<int>(w * roi_width_ratio_), 1, w);
  const int roi_h = std::clamp(static_cast<int>(h * roi_height_ratio_), 1, h);
  const int c0 = (w - roi_w) / 2;
  const int r0 = (h - roi_h) / 2;

  std::vector<float> ranges;
  ranges.reserve(static_cast<std::size_t>(roi_w) * roi_h);
  std::vector<std::array<int, 2>> pixels;
  pixels.reserve(static_cast<std::size_t>(roi_w) * roi_h);

  for (int r = r0; r < r0 + roi_h; ++r) {
    const uint8_t * row = img.data.data() + static_cast<std::size_t>(r) * img.step;
    for (int c = c0; c < c0 + roi_w; ++c) {
      float range;
      if (is_u16) {
        uint16_t mm;
        std::memcpy(&mm, row + static_cast<std::size_t>(c) * pixel_bytes, sizeof(mm));
        if (mm == 0) {continue;}
        range = static_cast<float>(mm) * 1e-3f;
      } else {
        std::memcpy(&range, row + static_cast<std::size_t>(c) * pixel_bytes, sizeof(range));
        if (!std::isfinite(range) || range <= 0.0f) {continue;}
      }
      if (range < static_cast<float>(min_range_) || range > static_cast<float>(max_range_)) {
        continue;
      }
      ranges.push_back(range);
      pixels.push_back({c, r});
    }
  }

  if (static_cast<int>(ranges.size()) < min_points_) {
    return out;  // ok = false
  }

  // Robust "nearest": the near_percentile-th smallest range.
  std::vector<float> sorted = ranges;
  const std::size_t k = std::min(
    sorted.size() - 1,
    static_cast<std::size_t>(near_percentile_ * static_cast<double>(sorted.size())));
  std::nth_element(sorted.begin(), sorted.begin() + k, sorted.end());
  const float near_range = sorted[k];

  // Centroid over pixels within depth_band of that nearest value.
  double su = 0.0;
  double sv = 0.0;
  double sd = 0.0;
  std::size_t n = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (ranges[i] <= near_range + static_cast<float>(depth_band_)) {
      su += pixels[i][0];
      sv += pixels[i][1];
      sd += ranges[i];
      ++n;
    }
  }
  if (static_cast<int>(n) < min_points_) {
    return out;
  }

  const double u_c = su / static_cast<double>(n);
  const double v_c = sv / static_cast<double>(n);
  const double d_c = sd / static_cast<double>(n);

  // Deproject (optical frame: x right, y down, z forward).
  out.x = (u_c - cx) * d_c / fx;
  out.y = (v_c - cy) * d_c / fy;
  out.z = d_c;
  out.distance = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
  out.ok = true;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[%s] centroid px=(%.0f,%.0f) d=%.2f -> optical(%.2f, %.2f, %.2f) dist=%.2f (n=%zu)",
    name_.c_str(), u_c, v_c, d_c, out.x, out.y, out.z, out.distance, n);
  return out;
}

}  // namespace custom_nav2_bt_plugins

PLUGINLIB_EXPORT_CLASS(
  custom_nav2_bt_plugins::DepthCentroidLocalizer,
  custom_nav2_bt_plugins::MovableObstacleLocalizer)
