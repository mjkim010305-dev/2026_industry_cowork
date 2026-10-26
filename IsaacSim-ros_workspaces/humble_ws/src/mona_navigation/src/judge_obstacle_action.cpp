#include "mona_navigation/judge_obstacle_action.hpp"

#include <memory>
#include <string>

namespace mona_navigation
{

JudgeObstacleAction::JudgeObstacleAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<mona_interfaces::action::JudgeObstacle>(
    xml_tag_name, action_name, conf)
{
}

void JudgeObstacleAction::on_tick()
{
  getInput("obstacle", goal_.obstacle);
}

BT::NodeStatus JudgeObstacleAction::on_success()
{
  setOutput("verdict", result_.result->verdict);
  setOutput("label", result_.result->label);
  setOutput("pushable", result_.result->pushable);
  RCLCPP_INFO(
    node_->get_logger(), "JudgeObstacle: verdict=%s label=\"%s\" votes=%u/%u latency=%.1fs (%s)",
    result_.result->verdict.c_str(), result_.result->label.c_str(),
    result_.result->votes_movable, result_.result->votes_n, result_.result->latency_s,
    result_.result->detail.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus JudgeObstacleAction::on_aborted()
{
  setOutput("verdict", mona_interfaces::action::JudgeObstacle::Result::ERROR);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus JudgeObstacleAction::on_cancelled()
{
  setOutput("verdict", mona_interfaces::action::JudgeObstacle::Result::ERROR);
  return BT::NodeStatus::FAILURE;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<mona_navigation::JudgeObstacleAction>(
        name, "judge_obstacle", config);
    };
  factory.registerBuilder<mona_navigation::JudgeObstacleAction>("JudgeObstacle", builder);
}
