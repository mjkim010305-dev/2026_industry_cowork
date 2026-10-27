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

#include "mona_navigation/movable_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "pluginlib/class_list_macros.hpp"

namespace mona_navigation
{

void MovableObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MovableObstacleLayer: node has gone away");
  }
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("list_topic", rclcpp::ParameterValue(std::string("/movable_obstacles/list")));
  // INSCRIBED (253): impassable for the planner, but distinguishable from
  // the lidar's LETHAL when read back. Set lower (e.g. 100) to let the
  // planner route through pushable objects.
  declareParameter("movable_cost", rclcpp::ParameterValue(253));
  declareParameter("fixed_cost", rclcpp::ParameterValue(254));
  declareParameter("mark_fixed", rclcpp::ParameterValue(true));
  // Extra metres drawn around the measured footprint.
  declareParameter("padding", rclcpp::ParameterValue(0.05));
  // false: reset() (e.g. Nav2 ClearEntireCostmap) keeps the objects, because
  // the publisher's list is authoritative and republished periodically.
  declareParameter("clear_on_reset", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".list_topic", list_topic_);
  int mc = 253, fc = 254;
  node->get_parameter(name_ + ".movable_cost", mc);
  node->get_parameter(name_ + ".fixed_cost", fc);
  node->get_parameter(name_ + ".mark_fixed", mark_fixed_);
  node->get_parameter(name_ + ".padding", padding_);
  node->get_parameter(name_ + ".clear_on_reset", clear_on_reset_);
  movable_cost_ = static_cast<unsigned char>(std::clamp(mc, 0, 254));
  fixed_cost_ = static_cast<unsigned char>(std::clamp(fc, 0, 254));

  sub_ = node->create_subscription<std_msgs::msg::String>(
    list_topic_, rclcpp::QoS(5),
    std::bind(&MovableObstacleLayer::onList, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    node->get_logger(), "MovableObstacleLayer: listening on %s (movable=%d fixed=%d mark_fixed=%s clear_on_reset=%s)",
    list_topic_.c_str(), mc, fc, mark_fixed_ ? "true" : "false",
    clear_on_reset_ ? "true" : "false");
}

void MovableObstacleLayer::onList(const std_msgs::msg::String::SharedPtr msg)
{
  std::vector<Obj> objs;
  std::string frame;
  if (!parse(msg->data, objs, frame)) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  objects_.swap(objs);
  list_frame_ = frame;
}

// The list is small and flat, so a targeted scan beats pulling in a JSON
// library the package does not otherwise need. Fields read per object:
// x, y, width_m, movable, pushable.
bool MovableObstacleLayer::parse(
  const std::string & s, std::vector<Obj> & out, std::string & frame)
{
  // json.dumps writes ": " by default; a value read straight after the colon
  // sees the space. That is how every object came out fixed (254) in the
  // first live test - skip blanks before reading anything.
  auto value_at = [&s](std::size_t from, const char * key) -> std::size_t {
      std::string k = std::string("\"") + key + "\":";
      auto p = s.find(k, from);
      if (p == std::string::npos) {return std::string::npos;}
      p += k.size();
      while (p < s.size() && (s[p] == ' ' || s[p] == '\t')) {++p;}
      return p;
    };
  auto num_after = [&](std::size_t from, const char * key, double & val) -> bool {
      auto p = value_at(from, key);
      if (p == std::string::npos) {return false;}
      val = std::strtod(s.c_str() + p, nullptr);
      return true;
    };
  auto bool_after = [&](std::size_t from, const char * key, bool & val) -> bool {
      auto p = value_at(from, key);
      if (p == std::string::npos) {return false;}
      val = s.compare(p, 4, "true") == 0;
      return true;
    };
  auto f = value_at(0, "frame_id");
  if (f != std::string::npos && s[f] == '"') {
    auto e = s.find('"', f + 1);
    frame = s.substr(f + 1, e - f - 1);
  }
  auto arr = s.find("\"objects\":");
  if (arr == std::string::npos) {return false;}
  std::size_t pos = arr;
  while (true) {
    auto open = s.find('{', pos);
    if (open == std::string::npos) {break;}
    auto close = s.find('}', open);
    if (close == std::string::npos) {break;}
    Obj o{0.0, 0.0, 0.0, false, false};
    double w = 0.0;
    bool ok = num_after(open, "x", o.x) && num_after(open, "y", o.y);
    num_after(open, "width_m", w);
    bool_after(open, "movable", o.movable);
    bool_after(open, "pushable", o.pushable);
    o.side = std::max(0.1, w);
    if (ok) {out.push_back(o);}
    pos = close + 1;
  }
  return true;
}

void MovableObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  double tx = 0.0, ty = 0.0, c = 1.0, sn = 0.0;
  const std::string & cf = layered_costmap_->getGlobalFrameID();
  if (!list_frame_.empty() && list_frame_ != cf) {
    try {
      auto tfm = tf_->lookupTransform(cf, list_frame_, tf2::TimePointZero);
      const auto & q = tfm.transform.rotation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      c = std::cos(yaw); sn = std::sin(yaw);
      tx = tfm.transform.translation.x; ty = tfm.transform.translation.y;
    } catch (const tf2::TransformException &) {
      return;
    }
  }
  // Both what will be drawn now and what was drawn last time, so stale
  // marks get refreshed away by the layers below.
  for (const auto * set : {&objects_, &last_drawn_}) {
    for (const auto & obj : *set) {
      const double x = c * obj.x - sn * obj.y + tx;
      const double y = sn * obj.x + c * obj.y + ty;
      double r = obj.side / 2.0 + padding_;
      *min_x = std::min(*min_x, x - r);
      *min_y = std::min(*min_y, y - r);
      *max_x = std::max(*max_x, x + r);
      *max_y = std::max(*max_y, y + r);
    }
  }
}

void MovableObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {return;}
  std::lock_guard<std::mutex> lock(mutex_);
  // The list is in the map frame; the local costmap lives in odom. Bring
  // the objects over with TF rather than refusing to draw.
  double tx = 0.0, ty = 0.0, c = 1.0, sn = 0.0;
  const std::string & cf = layered_costmap_->getGlobalFrameID();
  if (!list_frame_.empty() && list_frame_ != cf) {
    try {
      auto tfm = tf_->lookupTransform(cf, list_frame_, tf2::TimePointZero);
      const auto & q = tfm.transform.rotation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      c = std::cos(yaw); sn = std::sin(yaw);
      tx = tfm.transform.translation.x; ty = tfm.transform.translation.y;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000, "no TF %s <- %s: %s - not drawing",
        cf.c_str(), list_frame_.c_str(), ex.what());
      return;
    }
  }
  for (const auto & obj : objects_) {
    Obj o = obj;
    if (list_frame_ != cf) {
      o.x = c * obj.x - sn * obj.y + tx;
      o.y = sn * obj.x + c * obj.y + ty;
    }
    if (!o.movable && !mark_fixed_) {continue;}
    unsigned char cost = o.movable && o.pushable ? movable_cost_ : fixed_cost_;
    double r = o.side / 2.0 + padding_;
    unsigned int i0, j0, i1, j1;
    if (!master_grid.worldToMap(o.x - r, o.y - r, i0, j0)) {
      // Clamp the lower corner into the map rather than dropping the object.
      i0 = 0; j0 = 0;
      master_grid.worldToMapEnforceBounds(o.x - r, o.y - r, reinterpret_cast<int &>(i0), reinterpret_cast<int &>(j0));
    }
    master_grid.worldToMapEnforceBounds(o.x + r, o.y + r, reinterpret_cast<int &>(i1), reinterpret_cast<int &>(j1));
    for (int j = std::max<int>(j0, min_j); j <= std::min<int>(j1, max_j - 1); ++j) {
      for (int i = std::max<int>(i0, min_i); i <= std::min<int>(i1, max_i - 1); ++i) {
        // Pushable objects are re-marked DOWN from the lidar's LETHAL, so
        // this must overwrite, not max-combine.
        master_grid.setCost(i, j, cost);
      }
    }
  }
  last_drawn_ = objects_;
}

void MovableObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Kept objects stay in updateBounds (objects_ + last_drawn_), so the
  // cleared master grid gets them repainted on the next update.
  if (clear_on_reset_) {
    objects_.clear();
    last_drawn_.clear();
  }
  current_ = true;
}

}  // namespace mona_navigation

PLUGINLIB_EXPORT_CLASS(mona_navigation::MovableObstacleLayer, nav2_costmap_2d::Layer)
