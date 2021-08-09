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

#ifndef RTV_NAV2_PLAN_OPTIMIZER__RTV_NAV2_PLAN_OPTIMIZER_HPP_
#define RTV_NAV2_PLAN_OPTIMIZER__RTV_NAV2_PLAN_OPTIMIZER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rtv_nav2_plan_optimizer/plan_optimizer.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "tf2_ros/transform_listener.h"
#include "rtv_nav2_msgs/action/optimize_path.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rtv_nav2_plan_optimizer
{

/**
 * @class rtv_nav2_plan_optimizer::PlanOptimizerServer
 * @brief This class hosts variety of plugins of different algorithms to
 * complete control tasks from the exposed FollowPath action server.
 */
class PlanOptimizerServer : public nav2_util::LifecycleNode
{
public:
  using PlanOptimizerMap = std::unordered_map<std::string, rtv_nav2_plan_optimizer::PlanOptimizer::Ptr>;

  /**
   * @brief Constructor for rtv_nav2_plan_optimizer::PlanOptimizerServer
   */
  PlanOptimizerServer();
  /**
   * @brief Destructor for rtv_nav2_plan_optimizer::PlanOptimizerServer
   */
  ~PlanOptimizerServer();

protected:
  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * PlanOptimizer and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = rtv_nav2_msgs::action::OptimizePath;
  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the FollowPath action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  void optimizePlan();

  /**
   * @brief Wait for costmap to be valid with updated sensor data or repopulate after a
   * clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

  /**
   * @brief Find the valid controller ID name for the given request
   *
   * @param c_name The requested controller name
   * @param name Reference to the name to use for control if any valid available
   * @return bool Whether it found a valid controller to use
   */
  bool findPlanOptimizerId(const std::string & c_name, std::string & name);

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  void setPlannerPath(const nav_msgs::msg::Path & path);
  /**
   * @brief Calls setPlannerPath method with an updated path received from
   * action server
   */
  void updateGlobalPath();

  /**
   * @brief Obtain current pose of the robot
   * @param pose To store current pose of the robot
   * @return true if able to obtain current pose of the robot, else false
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  double poseDistance(const geometry_msgs::msg::TransformStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2);

  // The controller needs a costmap node
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers and subscribers
  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  
  // PlanOptimizer Plugins
  pluginlib::ClassLoader<rtv_nav2_plan_optimizer::PlanOptimizer> lp_loader_;
  PlanOptimizerMap optimizers_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> optimizer_ids_;
  std::vector<std::string> optimizer_types_;
  std::string optimizer_ids_concat_, current_plan_optimizer_;

  // Current path container
  nav_msgs::msg::Path whole_path_;
  nav_msgs::msg::Path current_path_;
  int current_path_begin_;
  int current_path_end_;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  double optimization_length_;
  double optimization_length_backwards_;
  double transform_tolerance_;
  double angular_distance_weight_;
  std::string robot_frame_id_;

};

}  // namespace rtv_nav2_plan_optimizer

#endif  // RTV_NAV2_PLAN_OPTIMIZER__RTV_NAV2_PLAN_OPTIMIZER_HPP_
