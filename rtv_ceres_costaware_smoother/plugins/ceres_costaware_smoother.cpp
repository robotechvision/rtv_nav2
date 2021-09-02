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

#include <algorithm>
#include <string>
#include <memory>
#include <utility>

#include "rtv_ceres_costaware_smoother/ceres_costaware_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace rtv_ceres_costaware_smoother
{

template<typename T>
inline int sign(T a) {
  return a == 0 ? 0 : a > 0 ? 1 : -1;
}

void CeresCostawareSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> & costmap_sub)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  
  double minimum_turning_radius;
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".minimum_turning_radius", rclcpp::ParameterValue(0.2));
  node->get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);

  _smoother = std::make_unique<rtv_ceres_costaware_smoother::Smoother>();
  _optimizer_params.get(node.get(), name);
  _smoother_params.get(node.get(), name);
  _smoother_params.max_curvature = 1.0f / minimum_turning_radius;
  _smoother_params.max_time = _optimizer_params.max_time;
  _smoother->initialize(_optimizer_params);

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".base_footprint_frame", rclcpp::ParameterValue("base_footprint"));
}

void CeresCostawareSmoother::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " rtv_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

void CeresCostawareSmoother::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "rtv_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

void CeresCostawareSmoother::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "rtv_ceres_costaware_smoother::CeresCostawareSmoother",
    plugin_name_.c_str());
}

nav_msgs::msg::Path CeresCostawareSmoother::smoothPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.size() <= 2)
    return path;

  auto costmap = costmap_sub_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());

  for (int i = 0; i < (int)path.poses.size()-1; i++) {
    auto &pose = path.poses[i].pose;
    auto &pos_next = path.poses[i+1].pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double angle = q.getAngle()*sign(q.getZ());
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);
    bool reversing = _smoother_params.reversing_enabled && orientation.dot(mvmt) < 0;
    path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
  }
  path_world.emplace_back(path.poses.back().pose.position.x, path.poses.back().pose.position.y, path_world.back()[2]);

  // Smooth plan
  if (!_smoother->smooth(path_world, &*costmap, _smoother_params)) {
    RCLCPP_WARN(
      logger_,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      plugin_name_.c_str());
    return path;
  }


  // populate final path
  nav_msgs::msg::Path plan;
  plan.header = path.header;
  plan.poses.reserve(path_world.size());
  plan.poses.push_back(path.poses.front());
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  for (uint i = 1; i < path_world.size()-1; i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    Eigen::Vector2d tangent_dir = tangentDir<double>(
      path_world[i-1].block<2, 1>(0, 0),
      path_world[i].block<2, 1>(0, 0),
      path_world[i+1].block<2, 1>(0, 0),
      path_world[i-1][2]*path_world[i][2]);
    Eigen::Vector2d current_dir = tangent_dir.dot((path_world[i+1] - path_world[i]).block<2, 1>(0, 0)*path_world[i][2]) >= 0 ? tangent_dir : -tangent_dir;
    double angle = atan2(current_dir[1], current_dir[0]);
    pose.pose.orientation.z = sin(angle/2);
    pose.pose.orientation.w = cos(angle/2);

    plan.poses.push_back(pose);
  }
  plan.poses.push_back(path.poses.back());

  return plan;
}

}  // namespace rtv_ceres_costaware_smoother

// Register this smoother as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  rtv_ceres_costaware_smoother::CeresCostawareSmoother,
  nav2_core::Smoother)
