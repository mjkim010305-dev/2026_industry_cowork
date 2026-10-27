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

#include "mona_navigation/path_length_within_condition.hpp"

#include "mona_navigation/push_geometry.hpp"

namespace mona_navigation
{

BT::NodeStatus PathLengthWithin::tick()
{
  nav_msgs::msg::Path path, reference;
  double ratio = 1.5, slack = 3.0;
  if (!getInput("path", path) || !getInput("reference", reference)) {
    return BT::NodeStatus::FAILURE;
  }
  getInput("ratio", ratio);
  getInput("slack", slack);
  return detail::path_length_within(path, reference, ratio, slack) ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace mona_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mona_navigation::PathLengthWithin>("PathLengthWithin");
}
