#ifndef MONA_NAVIGATION__PATH_OBSTACLE_DETECTED_CONDITION_HPP_
#define MONA_NAVIGATION__PATH_OBSTACLE_DETECTED_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "mona_interfaces/msg/path_obstacle.hpp"

namespace mona_navigation
{

/**
 * @brief SUCCESS iff the latest PathObstacle on `topic` has present == true and a
 * header stamp no older than `max_age`. The message is written to `obstacle`.
 * Geometry is computed by path_obstacle_monitor.py; this node only gates the tree.
 */
class PathObstacleDetectedCondition : public BT::ConditionNode
{
public:
  PathObstacleDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathObstacleDetectedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", std::string("/mona/path_obstacle"),
        "mona_interfaces/PathObstacle published by path_obstacle_monitor"),
      BT::InputPort<double>(
        "max_age", 1.0, "Max age [s] of the last message before it counts as stale"),
      BT::OutputPort<mona_interfaces::msg::PathObstacle>(
        "obstacle", "The detected obstacle (map frame)"),
      BT::OutputPort<std::string>(
        "sweep_count", "obstacle.sweep_count as a string (\"0\", \"1\", ...)"),
    };
  }

private:
  void callback(mona_interfaces::msg::PathObstacle::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<mona_interfaces::msg::PathObstacle>::SharedPtr sub_;

  std::mutex mutex_;
  mona_interfaces::msg::PathObstacle last_;
  bool received_ {false};
  double max_age_ {1.0};
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__PATH_OBSTACLE_DETECTED_CONDITION_HPP_
