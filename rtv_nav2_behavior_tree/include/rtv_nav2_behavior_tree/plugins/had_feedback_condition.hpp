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

#ifndef RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__HAD_FEEDBACK_CONDITION_HPP_
#define RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__HAD_FEEDBACK_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace rtv_nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to a feedback of a FollowPath action and
 * returns SUCCESS when the feedback has been received since last halt() and FAILURE otherwise
 */
class HadFeedbackCondition : public BT::ConditionNode
{
public:
  typedef nav2_msgs::action::FollowPath_FeedbackMessage MessageT;
  /**
   * @brief A constructor for rtv_nav2_behavior_tree::HadFeedbackCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  HadFeedbackCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  HadFeedbackCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "server_name", std::string("/follow_path"), "Action server name"),
    };
  }

private:
  /**
   * @brief Callback function for feedback topic
   * @param msg Shared pointer to nav2_msgs::action::FollowPath_FeedbackMessage message
   */
  void feedbackCallback(MessageT::SharedPtr msg);

  
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<MessageT>::SharedPtr feedback_sub_;
  std::string server_name_;
  bool had_feedback_;
};

}  // namespace rtv_nav2_behavior_tree

#endif  // RTV_NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__HAD_FEEDBACK_CONDITION_HPP_
