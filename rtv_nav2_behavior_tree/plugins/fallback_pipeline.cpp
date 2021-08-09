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

#include "rtv_nav2_behavior_tree/plugins/fallback_pipeline.hpp"

namespace rtv_nav2_behavior_tree
{

FallbackPipeline::FallbackPipeline(const std::string & name)
: BT::ControlNode(name, {})
{
}

FallbackPipeline::FallbackPipeline(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus FallbackPipeline::tick()
{
  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        if (i == 0) {
          ControlNode::haltChildren();
          last_child_ticked_ = 0;
          return status;
        }
        for (std::size_t j = i - 1; j <= last_child_ticked_; ++j) {
          children_nodes_[j]->halt();
        }
        last_child_ticked_ = i - 1;
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
        for (std::size_t j = i + 1; j <= last_child_ticked_; ++j) {
          children_nodes_[j]->halt();
        }
        break;
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          last_child_ticked_ = i;
          return status;
        }
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
  last_child_ticked_ = 0;
  return BT::NodeStatus::SUCCESS;
}

void FallbackPipeline::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace rtv_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rtv_nav2_behavior_tree::FallbackPipeline>("FallbackPipeline");
}
