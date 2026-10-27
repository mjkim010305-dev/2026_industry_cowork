#ifndef MONA_NAVIGATION__MARK_OBSTACLE_ACTION_HPP_
#define MONA_NAVIGATION__MARK_OBSTACLE_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "mona_interfaces/msg/path_obstacle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mona_navigation
{

/**
 * @brief Publish one PathObstacle once so obstacle_marks.py adds a temporary costmap mark.
 * FAILURE if the obstacle port is missing or present is false.
 */
class MarkObstacleAction : public BT::SyncActionNode
{
public:
  MarkObstacleAction(const std::string & name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<mona_interfaces::msg::PathObstacle>("obstacle", "Obstacle to mark"),
      BT::InputPort<std::string>("topic", "/mona/obstacle_marks/add", "Mark topic"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mona_interfaces::msg::PathObstacle>::SharedPtr pub_;
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__MARK_OBSTACLE_ACTION_HPP_
