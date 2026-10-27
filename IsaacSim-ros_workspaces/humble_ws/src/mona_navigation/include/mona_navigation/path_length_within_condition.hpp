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

#ifndef MONA_NAVIGATION__PATH_LENGTH_WITHIN_CONDITION_HPP_
#define MONA_NAVIGATION__PATH_LENGTH_WITHIN_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"

namespace mona_navigation
{

/// SUCCESS iff len(path) <= ratio * len(reference) + slack and path has >= 2 poses.
class PathLengthWithin : public BT::ConditionNode
{
public:
  PathLengthWithin(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Candidate path"),
      BT::InputPort<nav_msgs::msg::Path>("reference", "Reference path"),
      BT::InputPort<double>("ratio", 1.5, "Max length ratio to the reference"),
      BT::InputPort<double>("slack", 3.0, "Additive slack [m]"),
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace mona_navigation

#endif  // MONA_NAVIGATION__PATH_LENGTH_WITHIN_CONDITION_HPP_
