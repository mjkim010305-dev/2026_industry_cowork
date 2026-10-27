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

#ifndef MONA_NAVIGATION__PATH_CLEAR_IN_COSTMAP_CONDITION_HPP_
#define MONA_NAVIGATION__PATH_CLEAR_IN_COSTMAP_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "mona_navigation/costmap_geometry.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace mona_navigation
{

/**
 * SUCCESS iff no lethal local-costmap cell lies within `radius` of any pose in the first
 * `check_dist` metres of `path`. FAILURE if no costmap was received yet or TF fails.
 * Needs blackboard "node" and "tf_buffer".
 */
class PathClearInCostmap : public BT::ConditionNode
{
public:
  PathClearInCostmap(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to check (map frame)"),
      BT::InputPort<std::string>("costmap_topic", "/local_costmap/costmap_raw", "nav2_msgs/Costmap"),
      BT::InputPort<double>("check_dist", 3.0, "Check the first this-many metres of the path"),
      BT::InputPort<double>("radius", 0.22, "Clearance radius around each path pose [m]"),
      BT::InputPort<int>("lethal", 254, "Cell value counted as lethal"),
      BT::InputPort<std::string>("global_frame", "map", "Frame of the path"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr sub_;
  std::string topic_;

  std::mutex mutex_;
  detail::CostGrid grid_;
  std::string grid_frame_;
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__PATH_CLEAR_IN_COSTMAP_CONDITION_HPP_
