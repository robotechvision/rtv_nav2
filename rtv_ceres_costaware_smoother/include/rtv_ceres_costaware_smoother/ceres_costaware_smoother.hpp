// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#ifndef RTV_CERES_COSTAWARE_SMOOTHER__CERES_COSTAWARE_SMOOTHER_HPP_
#define RTV_CERES_COSTAWARE_SMOOTHER__CERES_COSTAWARE_SMOOTHER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/smoother.hpp"
#include "rtv_ceres_costaware_smoother/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace rtv_ceres_costaware_smoother
{

/**
 * @class rtv_ceres_costaware_smoother::CeresCostawareSmoother
 * @brief Regulated pure pursuit controller plugin
 */
class CeresCostawareSmoother : public nav2_core::Smoother
{
public:
  /**
   * @brief Constructor for rtv_ceres_costaware_smoother::CeresCostawareSmoother
   */
  CeresCostawareSmoother() = default;

  /**
   * @brief Destrructor for rtv_ceres_costaware_smoother::CeresCostawareSmoother
   */
  ~CeresCostawareSmoother() override = default;

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> & costmap_sub) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  nav_msgs::msg::Path smoothPath(
    const nav_msgs::msg::Path & path) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_ {rclcpp::get_logger("CeresCostawareSmoother")};

  std::unique_ptr<rtv_ceres_costaware_smoother::Smoother> _smoother;
  SmootherParams _smoother_params;
  OptimizerParams _optimizer_params;

};

}  // namespace rtv_ceres_costaware_smoother

#endif  // RTV_CERES_COSTAWARE_SMOOTHER__CERES_COSTAWARE_SMOOTHER_HPP_
