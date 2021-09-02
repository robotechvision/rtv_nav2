// Copyright (c) 2020 Sarthak Mittal
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

#include <string>

#include "rtv_nav2_behavior_tree/plugins/had_feedback_condition.hpp"

namespace rtv_nav2_behavior_tree
{

HadFeedbackCondition::HadFeedbackCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  server_name_("/follow_path"),
  had_feedback_(false)
{
  getInput("server_name", server_name_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  feedback_sub_ = node_->create_subscription<MessageT>(
    server_name_+"/_action/feedback",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&HadFeedbackCondition::feedbackCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus HadFeedbackCondition::tick()
{
  callback_group_executor_.spin_some();

  if (status() == BT::NodeStatus::IDLE) {
    setStatus(BT::NodeStatus::RUNNING);
    had_feedback_ = false;
  }

  if (had_feedback_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void HadFeedbackCondition::feedbackCallback(MessageT::SharedPtr msg)
{
  had_feedback_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rtv_nav2_behavior_tree::HadFeedbackCondition>("HadFeedback");
}
