// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020, Samsung Research America
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
// limitations under the License. Reserved.

#ifndef RTV_NAV2_SMAC_PLANNER__SMOOTHER_HPP_
#define RTV_NAV2_SMAC_PLANNER__SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "rtv_nav2_plan_optimizer/plugins/ceres_costaware_smoother/smoother_cost_function.hpp"

#include "ceres/ceres.h"
#include "Eigen/Core"

namespace rtv_nav2_plan_optimizer
{

/**
 * @class nav2_smac_planner::Smoother
 * @brief A Conjugate Gradient 2D path smoother implementation
 */
class Smoother
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::Smoother
   */
  Smoother() {}

  /**
   * @brief A destructor for nav2_smac_planner::Smoother
   */
  ~Smoother() {}

  /**
   * @brief Initialization of the smoother
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params)
  {
    _debug = params.debug;
    // General Params

    // 2 most valid options: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT
    _options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    _options.line_search_type = ceres::WOLFE;
    _options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE;
    _options.line_search_interpolation_type = ceres::CUBIC;

    _options.linear_solver_type = ceres::DENSE_QR;

    _options.max_num_iterations = params.max_iterations;
    _options.max_solver_time_in_seconds = params.max_time;

    _options.function_tolerance = params.fn_tol;
    _options.gradient_tolerance = params.gradient_tol;
    _options.parameter_tolerance = params.param_tol;

    _options.min_line_search_step_size = params.advanced.min_line_search_step_size;
    _options.max_num_line_search_step_size_iterations =
      params.advanced.max_num_line_search_step_size_iterations;
    _options.line_search_sufficient_function_decrease =
      params.advanced.line_search_sufficient_function_decrease;
    _options.max_line_search_step_contraction = params.advanced.max_line_search_step_contraction;
    _options.min_line_search_step_contraction = params.advanced.min_line_search_step_contraction;
    _options.max_num_line_search_direction_restarts =
      params.advanced.max_num_line_search_direction_restarts;
    _options.line_search_sufficient_curvature_decrease =
      params.advanced.line_search_sufficient_curvature_decrease;
    _options.max_line_search_step_expansion = params.advanced.max_line_search_step_expansion;

    if (_debug) {
      _options.minimizer_progress_to_stdout = true;
      _options.logging_type = ceres::LoggingType::PER_MINIMIZER_ITERATION;
    } else {
      _options.logging_type = ceres::SILENT;
    }
  }

  /**
   * @brief Smoother method
   * @param path Reference to path
   * @param costmap Pointer to minimal costmap
   * @param smoother parameters weights
   * @return If smoothing was successful
   */
  bool smooth(
    std::vector<Eigen::Vector3d> & path,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params)
  {
    //path always has at least 2 points
    CHECK(path.size() >= 2) << "Path must have at least 2 points";

    _options.max_solver_time_in_seconds = params.max_time;

    // double parameters[path.size() * 2];  // NOLINT
    // for (uint i = 0; i != path.size(); i++) {
    //   parameters[2 * i] = path[i][0];
    //   parameters[2 * i + 1] = path[i][1];
    // }

    ceres::Problem problem;

    const double dir_change_half_length = params.dir_change_length/2;
    ceres::LossFunction* loss_function = NULL;
    std::vector<Eigen::Vector3d> pathOptim = path;
    std::vector<bool> optimized(path.size());
    int prelast_i = -1;
    int last_i = 0;
    double last_direction = 1; // to avoid compiler warning, actually was_reversing is always initialized during the first iteration
    bool last_had_direction_change = false;
    bool last_is_reversing = false;
    std::deque<std::pair<double, SmootherCostFunction *>> potential_dir_change_funcs;
    double last_segment_len = EPSILON_DOUBLE;
    double current_segment_len = 0;
    double potential_dir_change_funcs_len = 0;
    double len_since_dir_change = std::numeric_limits<double>::infinity();
    for (int i = 0; i < (int)pathOptim.size(); i++) {
      auto &pt = pathOptim[i];

      // if (i > 0)
      //   current_segment_len += (pathOptim[i] - pathOptim[i-1]).block<2, 1>(0, 0).norm();

      bool direction_change = false;
      if (i != (int)pathOptim.size()-1) {
        // bool is_reversing = pt[2] < 0;//pt.rows() >= 3 && Eigen::Vector2d(cos(pt[2]), sin(pt[2])).dot((pathOptim[i+1] - pt).template block<2, 1>(0, 0)) < 0;
        // direction_change = was_reversing != is_reversing;
        direction_change = pt[2]*last_direction < 0;
        last_direction = pt[2];

        if (i == 0 || !direction_change && i > 1 && i < (int)pathOptim.size()-2 && (i - last_i) < params.input_downsampling_factor)
          continue;
      }

      current_segment_len += (pathOptim[i] - pathOptim[last_i]).block<2, 1>(0, 0).norm();

      potential_dir_change_funcs_len += current_segment_len;
      while (!potential_dir_change_funcs.empty() && potential_dir_change_funcs_len > dir_change_half_length) {
        potential_dir_change_funcs_len -= potential_dir_change_funcs.front().first;
        potential_dir_change_funcs.pop_front();
      }
      if (direction_change) {
        for (auto &f : potential_dir_change_funcs)
          f.second->setCostmapWeight(params.dir_change_costmap_weight);
        len_since_dir_change = 0;
        potential_dir_change_funcs_len = 0;
        potential_dir_change_funcs.clear();
      }

      optimized[i] = true;
      if (prelast_i != -1) {
        bool isDirChange = len_since_dir_change <= dir_change_half_length;
        SmootherCostFunction *cost_function = new SmootherCostFunction(
              path[last_i].template block<2, 1>(0, 0),
              (last_had_direction_change ? -1 : 1)*last_segment_len/current_segment_len, //(last_i - prelast_i)/(double)(i - last_i),
              last_is_reversing,
              costmap,
              params,
              isDirChange ? params.dir_change_costmap_weight : params.costmap_weight);
        problem.AddResidualBlock(cost_function->AutoDiff(), loss_function, pathOptim[last_i].data(), pt.data(), pathOptim[prelast_i].data());
        
        if (!isDirChange)
          potential_dir_change_funcs.emplace_back(current_segment_len, cost_function);
      }
      last_had_direction_change = direction_change;
      last_is_reversing = last_direction < 0;
      prelast_i = last_i;
      last_i = i;
      len_since_dir_change += current_segment_len;
      last_segment_len = std::max(EPSILON_DOUBLE, current_segment_len);
      current_segment_len = 0;
    }
    problem.SetParameterBlockConstant(pathOptim.front().data());
    if (pathOptim.size() >= 3) {
      problem.SetParameterBlockConstant(pathOptim[1].data());
      if (pathOptim.size() >= 4)
        problem.SetParameterBlockConstant(pathOptim[pathOptim.size()-2].data());
    }
    problem.SetParameterBlockConstant(pathOptim.back().data());

    // ceres::GradientProblemSolver::Summary summary;
    // ceres::GradientProblem problem(new SmootherCostFunction(&path, costmap, params));
    // ceres::Solve(_options, problem, parameters, &summary);

    ceres::Solver::Summary summary;
    ceres::Solve(_options, &problem, &summary);

    if (_debug) {
      RCLCPP_INFO(rclcpp::get_logger("planner_server"), "%s", summary.FullReport().c_str());
    }

    if (!summary.IsSolutionUsable() || summary.initial_cost - summary.final_cost <= 0.0) {
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("planner_server"), "Smoothing usable");
    path.resize(1); // keep the first point although not optimized
    for (int i = 1; i < (int)pathOptim.size(); i++)
      if (optimized[i])
        path.push_back(pathOptim[i]); // this will also add the last point
    // for (uint i = 0; i != path.size(); i++) {
    //   path[i][0] = parameters[2 * i];
    //   path[i][1] = parameters[2 * i + 1];
    // }

    return true;
  }

private:
  bool _debug;
  ceres::Solver::Options _options;
};

}  // namespace nav2_smac_planner

#endif  // RTV_NAV2_SMAC_PLANNER__SMOOTHER_HPP_
