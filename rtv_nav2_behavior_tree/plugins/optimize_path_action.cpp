// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "rtv_nav2_behavior_tree/plugins/optimize_path_action.hpp"

namespace rtv_nav2_behavior_tree
{

OptimizePathAction::OptimizePathAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<rtv_nav2_msgs::action::OptimizePath>(xml_tag_name, action_name, conf)
{
}

void OptimizePathAction::on_tick()
{
  getInput("path_in", goal_.path);
  getInput("optimizer_id", goal_.optimizer_id);
  if (getInput("start", goal_.start)) {
    goal_.use_start = true;
  }
}

BT::NodeStatus OptimizePathAction::on_success()
{
  setOutput("path_out", result_.result->path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rtv_nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<rtv_nav2_behavior_tree::OptimizePathAction>(
        name, "optimize_path", config);
    };

  factory.registerBuilder<rtv_nav2_behavior_tree::OptimizePathAction>(
    "OptimizePath", builder);
}
