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

#ifndef RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RESETTER_NODE_HPP_
#define RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RESETTER_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace rtv_nav2_behavior_tree
{
/**
 * @brief The ResetterNode returns result of its last child. If any of the other children
 * returns success, all children are halted, however, unlike in ReactiveFallback, the node does
 * not return success but keeps ticking the nodes until the last node returns success or failure.
 */
class ResetterNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for rtv_nav2_behavior_tree::ResetterNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ResetterNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);
  
  ResetterNode(const std::string & name);

  /**
   * @brief A destructor for rtv_nav2_behavior_tree::ResetterNode
   */
  ~ResetterNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

};

}  // namespace rtv_nav2_behavior_tree

#endif  // RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RESETTER_NODE_HPP_
