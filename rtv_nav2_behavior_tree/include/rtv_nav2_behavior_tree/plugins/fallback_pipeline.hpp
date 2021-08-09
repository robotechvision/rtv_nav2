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

#ifndef RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__FALLBACK_PIPELINE_HPP_
#define RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__FALLBACK_PIPELINE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace rtv_nav2_behavior_tree
{
/**
 * @brief Type of sequence node that re-ticks previous children when a child
 * returns running. If a ticked node returns failure, all nodes since the previous node
 * are halted and the execution is reset from the previous node.
 */
class FallbackPipeline : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for rtv_nav2_behavior_tree::FallbackPipeline
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  FallbackPipeline(
    const std::string & name,
    const BT::NodeConfiguration & conf);
  
  FallbackPipeline(const std::string & name);

  /**
   * @brief A destructor for rtv_nav2_behavior_tree::FallbackPipeline
   */
  ~FallbackPipeline() override = default;

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
};

}  // namespace rtv_nav2_behavior_tree

#endif  // RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__FALLBACK_PIPELINE_HPP_
