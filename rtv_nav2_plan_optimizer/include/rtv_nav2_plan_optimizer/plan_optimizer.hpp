// Copyright (c) 2021 RoboTech Vision
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

#ifndef RTV_NAV2_PLAN_OPTIMIZER__PLAN_OPTIMIZER_HPP_
#define RTV_NAV2_PLAN_OPTIMIZER__PLAN_OPTIMIZER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"


namespace rtv_nav2_plan_optimizer
{

/**
 * @class PlanOptimizer
 * @brief plan optimizer interface that acts as a virtual base class for all plan optimizer plugins
 */
class PlanOptimizer
{
public:
  using Ptr = std::shared_ptr<rtv_nav2_plan_optimizer::PlanOptimizer>;


  /**
   * @brief Virtual destructor
   */
  virtual ~PlanOptimizer() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> &,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> &) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  virtual nav_msgs::msg::Path optimizePath(
    const nav_msgs::msg::Path & path) = 0;
};

}  // namespace rtv_nav2_plan_optimizer

#endif  // RTV_NAV2_PLAN_OPTIMIZER__PLAN_OPTIMIZER_HPP_
