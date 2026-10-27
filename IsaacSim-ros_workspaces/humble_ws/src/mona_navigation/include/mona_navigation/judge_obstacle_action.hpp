#ifndef MONA_NAVIGATION__JUDGE_OBSTACLE_ACTION_HPP_
#define MONA_NAVIGATION__JUDGE_OBSTACLE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "mona_interfaces/action/judge_obstacle.hpp"
#include "mona_interfaces/msg/path_obstacle.hpp"

namespace mona_navigation
{

/**
 * @brief Send one PathObstacle to the JudgeObstacle action server (vlm_movability_node)
 * and expose the verdict/label on output ports. Action ABORTED/CANCELED -> FAILURE
 * with verdict "ERROR".
 */
class JudgeObstacleAction
  : public nav2_behavior_tree::BtActionNode<mona_interfaces::action::JudgeObstacle>
{
public:
  JudgeObstacleAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<mona_interfaces::msg::PathObstacle>("obstacle", "Obstacle to judge"),
        BT::OutputPort<std::string>("verdict", "MOVABLE | FIXED | UNSURE | NO_CANDIDATE | ERROR"),
        BT::OutputPort<std::string>("label", "VLM object label"),
        BT::OutputPort<bool>("pushable", "Depth size gate"),
        BT::OutputPort<std::string>("action", "PUSH | SWEEP | DETOUR"),
      });
  }
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__JUDGE_OBSTACLE_ACTION_HPP_
