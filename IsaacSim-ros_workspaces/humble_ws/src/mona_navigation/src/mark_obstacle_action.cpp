#include "mona_navigation/mark_obstacle_action.hpp"

namespace mona_navigation
{

MarkObstacleAction::MarkObstacleAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic = "/mona/obstacle_marks/add";
  getInput("topic", topic);
  pub_ = node_->create_publisher<mona_interfaces::msg::PathObstacle>(
    topic, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(node_->get_logger(), "MarkObstacle: topic \"%s\"", topic.c_str());
}

BT::NodeStatus MarkObstacleAction::tick()
{
  mona_interfaces::msg::PathObstacle obstacle;
  if (!getInput("obstacle", obstacle) || !obstacle.present) {
    return BT::NodeStatus::FAILURE;
  }
  pub_->publish(obstacle);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::MarkObstacleAction>("MarkObstacle");
}
