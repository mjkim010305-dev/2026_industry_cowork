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

#include "mona_navigation/path_clear_in_costmap_condition.hpp"

#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace mona_navigation
{

PathClearInCostmap::PathClearInCostmap(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  topic_ = "/local_costmap/costmap_raw";
  getInput("costmap_topic", topic_);
  {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = callback_group_;
    sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
      topic_, rclcpp::QoS(1).reliable(),
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
        detail::CostGrid g;
        g.size_x = static_cast<int>(m->metadata.size_x);
        g.size_y = static_cast<int>(m->metadata.size_y);
        g.res = m->metadata.resolution;
        g.ox = m->metadata.origin.position.x;
        g.oy = m->metadata.origin.position.y;
        g.data = m->data;
        std::lock_guard<std::mutex> lock(mutex_);
        grid_ = std::move(g);
        grid_frame_ = m->header.frame_id;
      },
      opts);
  }
}

BT::NodeStatus PathClearInCostmap::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  std::string global_frame = "map";
  double check_dist = 3.0, radius = 0.22;
  int lethal = 254;
  getInput("check_dist", check_dist);
  getInput("radius", radius);
  getInput("lethal", lethal);
  getInput("global_frame", global_frame);

  callback_group_executor_.spin_some();

  detail::CostGrid grid;
  std::string frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = grid_;
    frame = grid_frame_;
  }
  if (!grid.valid()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "PathClearInCostmap: no costmap on \"%s\" yet", topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::string path_frame = path.header.frame_id.empty() ? global_frame : path.header.frame_id;
  tf2::Transform t;   // path frame -> costmap frame
  try {
    const auto tf = tf_buffer_->lookupTransform(frame, path_frame, tf2::TimePointZero);
    tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
      tf.transform.rotation.w);
    t = tf2::Transform(
      q, tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "PathClearInCostmap: TF %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::pair<double, double>> pts;
  pts.reserve(path.poses.size());
  for (const auto & ps : path.poses) {
    const tf2::Vector3 p = t * tf2::Vector3(ps.pose.position.x, ps.pose.position.y, 0.0);
    pts.emplace_back(p.x(), p.y());
  }
  const bool clear = detail::path_clear_in_grid(
    grid, pts, check_dist, radius, static_cast<uint8_t>(lethal));
  if (!clear) {
    RCLCPP_INFO(node_->get_logger(), "PathClearInCostmap: lethal cell within %.2f m of the path", radius);
  }
  return clear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::PathClearInCostmap>("PathClearInCostmap");
}
