#include "mona_navigation/path_obstacle_detected_condition.hpp"

#include <functional>

namespace mona_navigation
{

PathObstacleDetectedCondition::PathObstacleDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic = "/mona/path_obstacle";
  getInput("topic", topic);
  getInput("max_age", max_age_);

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  sub_ = node_->create_subscription<mona_interfaces::msg::PathObstacle>(
    topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&PathObstacleDetectedCondition::callback, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(
    node_->get_logger(), "PathObstacleDetected: topic \"%s\", max_age=%.2fs",
    topic.c_str(), max_age_);
}

void PathObstacleDetectedCondition::callback(mona_interfaces::msg::PathObstacle::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_ = *msg;
  received_ = true;
}

BT::NodeStatus PathObstacleDetectedCondition::tick()
{
  callback_group_executor_.spin_some();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_ || !last_.present) {
    return BT::NodeStatus::FAILURE;
  }
  const rclcpp::Time stamp(last_.header.stamp, node_->get_clock()->get_clock_type());
  const double age = (node_->now() - stamp).seconds();
  if (age < 0.0 || age > max_age_) {
    return BT::NodeStatus::FAILURE;
  }
  if (!gate_.accept(last_.header.stamp.sec, last_.header.stamp.nanosec)) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("obstacle", last_);
  setOutput("sweep_count", std::to_string(last_.sweep_count));
  setOutput("push_count", std::to_string(last_.push_count));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::PathObstacleDetectedCondition>(
    "PathObstacleDetected");
}
