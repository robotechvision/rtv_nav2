// Copyright (c) 2019 RoboTech Vision
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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rtv_nav2_plan_optimizer/rtv_nav2_plan_optimizer.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

namespace rtv_nav2_plan_optimizer
{

PlanOptimizerServer::PlanOptimizerServer()
: LifecycleNode("optimizer_server", "", true),
  lp_loader_("rtv_nav2_plan_optimizer", "rtv_nav2_plan_optimizer::PlanOptimizer"),
  default_ids_{"OptimizePath"},
  default_types_{"rtv_nav2_plan_optimizer::CeresCostawareSmoother"}
{
  RCLCPP_INFO(get_logger(), "Creating plan optimizer server");

  declare_parameter("optimizer_plugins", default_ids_);
  declare_parameter("optimization_length", rclcpp::ParameterValue(6.0));
  declare_parameter("optimization_length_backwards", rclcpp::ParameterValue(3.0));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter("robot_frame_id", rclcpp::ParameterValue("base_footprint"));
  declare_parameter("angular_distance_weight", rclcpp::ParameterValue(0.2));

  // // Launch a thread to run the costmap node
  // costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  RCLCPP_INFO(get_logger(), "Optimizer server created");
}

PlanOptimizerServer::~PlanOptimizerServer()
{
  optimizers_.clear();
  // costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlanOptimizerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();

  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("optimizer_plugins", optimizer_ids_);
  get_parameter("optimization_length", optimization_length_);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("robot_frame_id", robot_frame_id_);
  get_parameter("angular_distance_weight", angular_distance_weight_);
  get_parameter("optimization_length_backwards", optimization_length_backwards_);
  if (optimizer_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp_node_->get_node_base_interface(),
    rclcpp_node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // The costmap node is used in the implementation of the controller
  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    shared_from_this(), "global_costmap/costmap_raw");
  
  optimizer_types_.resize(optimizer_ids_.size());
  // costmap_ros_->on_configure(state);

  for (size_t i = 0; i != optimizer_ids_.size(); i++) {
    try {
      optimizer_types_[i] = nav2_util::get_plugin_type_param(node, optimizer_ids_[i]);
      rtv_nav2_plan_optimizer::PlanOptimizer::Ptr optimizer =
        lp_loader_.createUniqueInstance(optimizer_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created plan optimizer : %s of type %s",
        optimizer_ids_[i].c_str(), optimizer_types_[i].c_str());
      optimizer->configure(
        node, optimizer_ids_[i],
        tf_buffer_, costmap_sub_);
      optimizers_.insert({optimizer_ids_[i], optimizer});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(),
        "Failed to create plan optimizer. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != optimizer_ids_.size(); i++) {
    optimizer_ids_concat_ += optimizer_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Plan optimizer Server has %s optimizers available.", optimizer_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan_optimized", 1);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    rclcpp_node_, "optimize_path",
    std::bind(&PlanOptimizerServer::optimizePlan, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlanOptimizerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  RCLCPP_INFO(get_logger(), "Checking transform");
  rclcpp::Rate r(2);

  plan_publisher_->on_activate();  
  PlanOptimizerMap::iterator it;
  for (it = optimizers_.begin(); it != optimizers_.end(); ++it) {
    it->second->activate();
  }
  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlanOptimizerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();
  PlanOptimizerMap::iterator it;
  for (it = optimizers_.begin(); it != optimizers_.end(); ++it) {
    it->second->deactivate();
  }
  plan_publisher_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlanOptimizerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Cleanup the helper classes
  PlanOptimizerMap::iterator it;
  for (it = optimizers_.begin(); it != optimizers_.end(); ++it) {
    it->second->cleanup();
  }
  optimizers_.clear();

  // Release any allocated resources
  action_server_.reset();
  odom_sub_.reset();
  action_server_.reset();
  plan_publisher_.reset();
  tf_buffer_.reset();
  tf_listener_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlanOptimizerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool PlanOptimizerServer::findPlanOptimizerId(
  const std::string & c_name,
  std::string & current_plan_optimizer)
{
  if (optimizers_.find(c_name) == optimizers_.end()) {
    if (optimizers_.size() == 1 && c_name.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No controller was specified in action call."
        " Server will use only plugin loaded %s. "
        "This warning will appear once.", optimizer_ids_concat_.c_str());
      current_plan_optimizer = optimizers_.begin()->first;
    } else {
      RCLCPP_ERROR(
        get_logger(), "FollowPath called with controller name %s, "
        "which does not exist. Available controllers are: %s.",
        c_name.c_str(), optimizer_ids_concat_.c_str());
      return false;
    }
  } else {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_plan_optimizer = c_name;
  }

  return true;
}

void PlanOptimizerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (true) {
    try {
      costmap_sub_->getCostmap();
      break;
    }
    catch (std::runtime_error e) {
    }
  }

  // while (!costmap_ros_->isCurrent()) {
  //   r.sleep();
  // }
}

void PlanOptimizerServer::optimizePlan()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  auto result = std::make_shared<Action::Result>();
  try {
    std::string c_name = action_server_->get_current_goal()->optimizer_id;
    std::string current_plan_optimizer;
    if (findPlanOptimizerId(c_name, current_plan_optimizer)) {
      current_plan_optimizer_ = current_plan_optimizer;
    } else {
      action_server_->terminate_current();
      return;
    }

    // waitForCostmap();

    setPlannerPath(action_server_->get_current_goal()->path);

    // updateGlobalPath();
    auto start_time = steady_clock_.now();

    result->path = whole_path_;
    result->path.poses.erase(result->path.poses.begin() + current_path_begin_, result->path.poses.begin() + current_path_end_);
    auto optimized_current_path = optimizers_[current_plan_optimizer_]->optimizePath(current_path_).poses;
    result->path.poses.insert(result->path.poses.begin() + current_path_begin_, optimized_current_path.begin(), optimized_current_path.end());
    result->path.header.stamp = now();

    auto cycle_duration = steady_clock_.now() - start_time;
    result->optimization_time = cycle_duration;

    RCLCPP_INFO(get_logger(), "PlanOptimizer succeeded (time: %lf), setting result", rclcpp::Duration(result->optimization_time).seconds());
    plan_publisher_->publish(result->path);

    action_server_->succeeded_current(result);
  } catch (nav2_core::PlannerException & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    action_server_->terminate_current();
    return;
  }
}

double PlanOptimizerServer::poseDistance(const geometry_msgs::msg::TransformStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2) {
  double dx = pose1.transform.translation.x - pose2.pose.position.x;
  double dy = pose1.transform.translation.y - pose2.pose.position.y;
  tf2::Quaternion q1;
  tf2::convert(pose1.transform.rotation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight_*std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

void PlanOptimizerServer::setPlannerPath(const nav_msgs::msg::Path & path)
{
  RCLCPP_DEBUG(
    get_logger(),
    "Providing path to the controller %s", current_plan_optimizer_.c_str());
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  // optimizers_[current_plan_optimizer_]->setPlan(path);

  whole_path_ = path;
  if (optimization_length_ > 0) {
    // auto end_pose = path.poses.back();
    // end_pose.header.frame_id = path.header.frame_id;
    rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(transform_tolerance_));

    rclcpp::Time now = this->now();
    // tf_buffer_->waitForTransform(costmap_sub_->getHeader().frame_id, robot_frame_id_, now, tolerance);
    try {
      geometry_msgs::msg::TransformStamped robot_pose = tf_buffer_->lookupTransform(costmap_sub_->getHeader().frame_id, robot_frame_id_, now, tolerance);

      auto current_pose =
        nav2_util::geometry_utils::min_by(
        path.poses.begin(), path.poses.end(),
        [&robot_pose, this](const geometry_msgs::msg::PoseStamped & ps) {
          return poseDistance(robot_pose, ps);
        });
      
      double length = 0;
      current_path_end_ = current_pose - path.poses.begin();
      while (current_path_end_ < (int)path.poses.size()-1 && length < optimization_length_) {
        length += std::hypot(path.poses[current_path_end_+1].pose.position.x - path.poses[current_path_end_].pose.position.x,
                             path.poses[current_path_end_+1].pose.position.y - path.poses[current_path_end_].pose.position.y);
        current_path_end_++;
      }
      current_path_end_++; // end is exclusive

      current_path_begin_ = current_pose - path.poses.begin();
      length = 0;
      while (current_path_begin_ > 0 && length < optimization_length_backwards_) {
        length += std::hypot(path.poses[current_path_begin_+1].pose.position.x - path.poses[current_path_begin_].pose.position.x,
                             path.poses[current_path_begin_+1].pose.position.y - path.poses[current_path_begin_].pose.position.y);
        current_path_begin_--;
      }

      ////TODO: delete////////////
      length = 0;
      int i = current_path_begin_;
      while (i < current_path_end_-1) {
        length += std::hypot(path.poses[i+1].pose.position.x - path.poses[i].pose.position.x,
                             path.poses[i+1].pose.position.y - path.poses[i].pose.position.y);
        i++;
      }
      RCLCPP_INFO(
        get_logger(),
        "Path length %lf", length);
      ////////////////////////////

      current_path_.header = path.header;
      current_path_.poses = std::vector<geometry_msgs::msg::PoseStamped>(path.poses.begin() + current_path_begin_, path.poses.begin() + current_path_end_);
    } catch (tf2::TransformException &ex) {
      std::stringstream ss;
      ss << "Could not find transform between " << robot_frame_id_ << " and " << costmap_sub_->getHeader().frame_id << ": " << ex.what();
      throw nav2_core::PlannerException(ss.str());
    }
  }
  else
    current_path_ = path;
}

void PlanOptimizerServer::updateGlobalPath()
{
  if (action_server_->is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Passing new path to controller.");
    auto goal = action_server_->accept_pending_goal();
    std::string current_plan_optimizer;
    if (findPlanOptimizerId(goal->optimizer_id, current_plan_optimizer)) {
      current_plan_optimizer_ = current_plan_optimizer;
    } else {
      RCLCPP_INFO(
        get_logger(), "Terminating action, invalid controller %s requested.",
        goal->optimizer_id.c_str());
      action_server_->terminate_current();
      return;
    }
    setPlannerPath(goal->path);
  }
}

bool PlanOptimizerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  // geometry_msgs::msg::PoseStamped current_pose;
  // if (!costmap_ros_->getRobotPose(current_pose)) {
  //   return false;
  // }
  // pose = current_pose;
  // return true;
  return false;
}

}  // namespace rtv_nav2_plan_optimizer
