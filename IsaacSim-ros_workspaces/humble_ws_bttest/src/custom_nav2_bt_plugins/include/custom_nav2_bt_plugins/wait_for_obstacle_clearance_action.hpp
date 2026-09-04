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

#ifndef CUSTOM_NAV2_BT_PLUGINS__WAIT_FOR_OBSTACLE_CLEARANCE_ACTION_HPP_
#define CUSTOM_NAV2_BT_PLUGINS__WAIT_FOR_OBSTACLE_CLEARANCE_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace custom_nav2_bt_plugins
{

/**
 * @brief A BT::StatefulActionNode that subscribes to a std_msgs/Bool topic
 * reporting whether a previously detected obstacle is still present.
 *
 * Semantics:
 *  - `onStart()` returns RUNNING and starts waiting;
 *  - while the topic reports `true` (obstacle still there) the node returns
 *    RUNNING, so the robot holds its position;
 *  - when the topic reports `false` (obstacle handled / removed) it returns
 *    SUCCESS;
 *  - if `clear_on_stale` is true and messages stop arriving for longer than
 *    `message_timeout`, that is also treated as "cleared" (SUCCESS);
 *  - if `timeout` > 0 and no clearance is seen within that many seconds, the
 *    node gives up waiting and returns SUCCESS anyway so normal navigation
 *    (and the standard recovery behaviors) can resume.
 *
 * The node never returns FAILURE, so it can be used behind an Inverter in a
 * ReactiveSequence without ever collapsing the surrounding fallback to SUCCESS.
 */
class WaitForObstacleClearanceAction : public BT::StatefulActionNode
{
public:
  WaitForObstacleClearanceAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  WaitForObstacleClearanceAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", std::string("/obstacle/still_present"),
        "std_msgs/Bool topic from the VLM: true while the obstacle is still there"),
      BT::InputPort<double>(
        "timeout", 0.0,
        "If > 0, stop waiting and return SUCCESS after this many seconds"),
      BT::InputPort<bool>(
        "clear_on_stale", true,
        "Treat missing / stale messages as 'obstacle cleared'"),
      BT::InputPort<double>(
        "message_timeout", 3.0,
        "Maximum age [s] of the last message before it is considered stale"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void presentCallback(std_msgs::msg::Bool::SharedPtr msg);
  BT::NodeStatus evaluate();

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr present_sub_;

  std::mutex mutex_;
  bool obstacle_present_;
  bool msg_received_;
  rclcpp::Time last_msg_time_;

  std::string topic_;
  double timeout_;
  bool clear_on_stale_;
  double message_timeout_;
  rclcpp::Time start_time_;
};

}  // namespace custom_nav2_bt_plugins

#endif  // CUSTOM_NAV2_BT_PLUGINS__WAIT_FOR_OBSTACLE_CLEARANCE_ACTION_HPP_
