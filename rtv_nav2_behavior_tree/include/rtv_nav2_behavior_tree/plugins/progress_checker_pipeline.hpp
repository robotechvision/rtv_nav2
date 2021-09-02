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

#ifndef RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PROGRESS_CHECKER_PIPELINE_HPP_
#define RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PROGRESS_CHECKER_PIPELINE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace rtv_nav2_behavior_tree
{
/**
 * @brief Type of sequence node that re-ticks previous children when a child
 * returns running. If a ticked node returns failure, all nodes since the previous node
 * are halted and the execution is restarted from the previous node. The last node is
 * a progress condition which in the case of a failure of a node checks whether
 * progress has been made since last failure. If no progress has been detected,
 * the execution is restarted from the last node that hasn't been halted since the no-progress
 * situation began. If this restart fails without progress too, next restart starts from the
 * previous node and so on, until the first node is reached. If the restart from the first node
 * fails without progress, ProgressCheckerPipeline returns failure.
 */
class ProgressCheckerPipeline : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for rtv_nav2_behavior_tree::ProgressCheckerPipeline
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ProgressCheckerPipeline(
    const std::string & name,
    const BT::NodeConfiguration & conf);
  
  ProgressCheckerPipeline(const std::string & name);

  /**
   * @brief A destructor for rtv_nav2_behavior_tree::ProgressCheckerPipeline
   */
  ~ProgressCheckerPipeline() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {};
  }

  void halt() override;

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
  int last_child_with_progress_ = -1;
};

}  // namespace rtv_nav2_behavior_tree

#endif  // RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PROGRESS_CHECKER_PIPELINE_HPP_
