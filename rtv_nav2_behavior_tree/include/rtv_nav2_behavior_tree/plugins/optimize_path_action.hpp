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

#ifndef RTV_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OPTIMIZE_PATH_ACTION_HPP_
#define RTV_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OPTIMIZE_PATH_ACTION_HPP_

#include <string>

#include "rtv_nav2_msgs/action/optimize_path.hpp"
#include "nav_msgs/msg/path.h"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace rtv_nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps rtv_nav2_msgs::action::OptimizePath
 */
class OptimizePathAction : public nav2_behavior_tree::BtActionNode<rtv_nav2_msgs::action::OptimizePath>
{
public:
  /**
   * @brief A constructor for rtv_nav2_behavior_tree::OptimizePathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  OptimizePathAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<nav_msgs::msg::Path>("path_out", "Path smoothed by PlanOptimizerServer node"),
        BT::InputPort<nav_msgs::msg::Path>("path_in", "Path to be smoothed"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
          "start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("optimizer_id", ""),
      });
  }
};

}  // namespace rtv_nav2_behavior_tree

#endif  // RTV_NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__OPTIMIZE_PATH_ACTION_HPP_
