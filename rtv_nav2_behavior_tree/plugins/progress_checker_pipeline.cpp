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

#include "rtv_nav2_behavior_tree/plugins/progress_checker_pipeline.hpp"

namespace rtv_nav2_behavior_tree
{

ProgressCheckerPipeline::ProgressCheckerPipeline(const std::string & name)
: BT::ControlNode(name, {})
{
}

ProgressCheckerPipeline::ProgressCheckerPipeline(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus ProgressCheckerPipeline::tick()
{
  auto progress_status = children_nodes_.back()->executeTick();

  for (std::size_t i = 0; i < children_nodes_.size()-1; ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        RCLCPP_INFO(rclcpp::get_logger("progress_checker_pipeline"), "(before failure) last_child_ticked: %d, last_child_with_progress: %d, progress: %d",
                    (int)last_child_ticked_, (int)last_child_with_progress_, (int)(progress_status == BT::NodeStatus::SUCCESS));
        if (i == 0 || (progress_status == BT::NodeStatus::FAILURE && last_child_with_progress_ == -1)) {
          ControlNode::haltChildren();
          last_child_ticked_ = 0;
          return status;
        }
        {
          std::size_t new_last_child_ticked = i - 1;
          if (progress_status == BT::NodeStatus::FAILURE) {
            new_last_child_ticked = std::min((std::size_t)last_child_with_progress_, new_last_child_ticked);
          }
          last_child_with_progress_ = (int)new_last_child_ticked - 1;

          for (std::size_t j = new_last_child_ticked; j <= last_child_ticked_; ++j) {
            children_nodes_[j]->halt();
          }
          children_nodes_.back()->halt();
          last_child_ticked_ = new_last_child_ticked;
          RCLCPP_INFO(rclcpp::get_logger("progress_checker_pipeline"), "FAILURE: last_child_ticked: %d, last_child_with_progress: %d, progress: %d",
                      (int)last_child_ticked_, (int)last_child_with_progress_, (int)(progress_status == BT::NodeStatus::SUCCESS));
        }

        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
        for (std::size_t j = i + 1; j <= last_child_ticked_; ++j) {
          children_nodes_[j]->halt();
        }
        break;
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          bool changed = i != last_child_ticked_;
          if (changed)
            RCLCPP_INFO(rclcpp::get_logger("progress_checker_pipeline"), "(before) last_child_ticked: %d, last_child_with_progress: %d, progress: %d",
                        (int)last_child_ticked_, (int)last_child_with_progress_, (int)(progress_status == BT::NodeStatus::SUCCESS));
          last_child_ticked_ = i;
          if (progress_status == BT::NodeStatus::SUCCESS)
            last_child_with_progress_ = (int)i;
          if (changed)
            RCLCPP_INFO(rclcpp::get_logger("progress_checker_pipeline"), "last_child_ticked: %d, last_child_with_progress: %d, progress: %d",
                        (int)last_child_ticked_, (int)last_child_with_progress_, (int)(progress_status == BT::NodeStatus::SUCCESS));
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

void ProgressCheckerPipeline::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace rtv_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rtv_nav2_behavior_tree::ProgressCheckerPipeline>("ProgressCheckerPipeline");
}
