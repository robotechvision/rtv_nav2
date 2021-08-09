// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2019 Intel Corporation
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

#include <stdexcept>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "rtv_nav2_behavior_tree/plugins/resetter_node.hpp"

namespace rtv_nav2_behavior_tree
{

ResetterNode::ResetterNode(const std::string & name)
: BT::ControlNode(name, {})
{
}

ResetterNode::ResetterNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus ResetterNode::tick()
{
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    if (i == children_nodes_.size()-1)
      return status;
    else
      switch (status) {
        case BT::NodeStatus::FAILURE:
          break;
        case BT::NodeStatus::SUCCESS:
          ControlNode::haltChildren();
          break;
        case BT::NodeStatus::RUNNING:
          break;
        default:
          std::stringstream error_msg;
          error_msg << "Invalid node status. Received status " << status <<
            "from child " << children_nodes_[i]->name();
          throw std::runtime_error(error_msg.str());
      }
  }
  // Wrap up.
  ControlNode::haltChildren();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtv_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rtv_nav2_behavior_tree::ResetterNode>("ResetterNode");
}
