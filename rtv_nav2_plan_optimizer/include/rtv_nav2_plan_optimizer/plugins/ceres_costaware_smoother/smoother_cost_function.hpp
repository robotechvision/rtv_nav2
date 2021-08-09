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

#ifndef RTV_NAV2_SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_
#define RTV_NAV2_SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_

#include <cmath>
#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>
#include <queue>
#include <utility>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "Eigen/Core"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rtv_nav2_plan_optimizer/plugins/ceres_costaware_smoother/options.hpp"

#define EPSILON_DOUBLE 0.0001
#define EPSILON (T)EPSILON_DOUBLE

namespace rtv_nav2_plan_optimizer
{

template <typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(Eigen::Matrix<T, 2, 1> p1, Eigen::Matrix<T, 2, 1> p, Eigen::Matrix<T, 2, 1> p2, int forced_dot_sign = 0) {
  Eigen::Matrix<T, 2, 1> d1 = p - p1;
  Eigen::Matrix<T, 2, 1> d2 = p2 - p;
  T det = d1[0]*d2[1] - d1[1]*d2[0];
  
  if (forced_dot_sign < 0 || forced_dot_sign == 0 && d1.dot(d2) < (T)0) {
    // changed from forward movement to reverse or vice versa - use average direction towards p
    Eigen::Matrix<T, 2, 1> tangent_dir = d1.norm()*d1 - d2.norm()*d2;
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "p1: %lf %lf, p: %lf %lf, p2: %lf %lf, dir changed: %lf %lf",
    //         p1[0], p1[1], p[0], p[1], p2[0], p2[1], tangent_dir[0], tangent_dir[1]);

    return tangent_dir;
  }

  if (ceres::abs(det) < (T)1e-4) { // straight line
    Eigen::Matrix<T, 2, 1> tangent_dir(p2[0] - p1[0], p2[1] - p1[1]);
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "p1: %lf %lf, p: %lf %lf, p2: %lf %lf, straight line: %lf %lf",
    //             p1[0], p1[1], p[0], p[1], p2[0], p2[1], tangent_dir[0], tangent_dir[1]);
    return tangent_dir;
  }

  // circle center is at the intersection of the mirror axes of the segments: http://paulbourke.net/geometry/circlesphere/
  // intersection: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Intersection%20of%20two%20lines
  Eigen::Matrix<T, 2, 1> mid1 = (p1 + p)/(T)2;
  Eigen::Matrix<T, 2, 1> mid2 = (p + p2)/(T)2;
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "det before: %lf", det);
  // det = n1[0]*n2[1] - n1[1]*n2[0];
  T det1 = (mid1[0] + n1[0])*mid1[1] - (mid1[1] + n1[1])*mid1[0];
  T det2 = (mid2[0] + n2[0])*mid2[1] - (mid2[1] + n2[1])*mid2[0];
  Eigen::Matrix<T, 2, 1> center((det1*n2[0] - det2*n1[0])/det, (det1*n2[1] - det2*n1[1])/det);
  Eigen::Matrix<T, 2, 1> tangent_dir(center[1] - p[1], p[0] - center[0]);
  // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "p1: %lf %lf, p: %lf %lf, p2: %lf %lf, det: %lf, det1: %lf, det2: %lf center: %lf %lf, tangent: %lf %lf",
  //             p1[0], p1[1], p[0], p[1], p2[0], p2[1], det, det1, det2, center[0], center[1], tangent_dir[0], tangent_dir[1]);
  return tangent_dir; // prependicular to (p - center)
}

/**
 * @struct nav2_smac_planner::SmootherCostFunction
 * @brief Cost function for path smoothing with multiple terms
 * including curvature, smoothness, collision, and avoid obstacles.
 */
class SmootherCostFunction
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::SmootherCostFunction
   * @param original_path Original unsmoothed path to smooth
   * @param costmap A costmap to get values for collision and obstacle avoidance
   */
  SmootherCostFunction(
    const Eigen::Vector2d &_original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    nav2_costmap_2d::Costmap2D * costmap,
    const SmootherParams & params,
    double costmap_weight)
  : _original_pos(_original_pos),
    _next_to_last_length_ratio(next_to_last_length_ratio),
    _reversing(reversing),
    _costmap(costmap),
    _params(params),
    _costmap_weight(costmap_weight)
  {
    costmapGrid.reset(new ceres::Grid2D<u_char>(costmap->getCharMap(), 0, costmap->getSizeInCellsY(), 0, costmap->getSizeInCellsX()));
    interpolateCostmap.reset(new ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>(*costmapGrid));
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "x: %lf, y: %lf, ntllr: %lf, rev: %d, cost_w: %lf",
    //             _original_pos[0], _original_pos[1], _next_to_last_length_ratio, (int)_reversing, _costmap_weight);
  }

  ceres::CostFunction* AutoDiff() {
    return (new ceres::AutoDiffCostFunction<SmootherCostFunction, 4, 2, 2, 2>(this));
  }

  void setCostmapWeight(double costmap_weight) {
    _costmap_weight = costmap_weight;
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "updated x: %lf, y: %lf, ntllr: %lf, rev: %d, cost_w: %lf",
    //             _original_pos[0], _original_pos[1], _next_to_last_length_ratio, (int)_reversing, _costmap_weight);
  }

  /**
   * @struct CurvatureComputations
   * @brief Cache common computations between the curvature terms to minimize recomputations
   */
  template <typename T>
  struct CurvatureComputations
  {
    /**
     * @brief A constructor for nav2_smac_planner::CurvatureComputations
     */
    CurvatureComputations()
    {
      valid = true;
    }

    bool valid;
    /**
     * @brief Check if result is valid for penalty
     * @return is valid (non-nan, non-inf, and turning angle > max)
     */
    bool isValid()
    {
      return valid;
    }

    Eigen::Matrix<T, 2, 1> delta_xi{0.0, 0.0};
    Eigen::Matrix<T, 2, 1> delta_xi_p{0.0, 0.0};
    T delta_xi_norm{0};
    T delta_xi_p_norm{0};
    T delta_phi_i{0};
    T turning_rad{0};
    T ki_minus_kmax{0};
  };

  /**
   * @brief Smoother cost function evaluation
   * @param parameters X,Y pairs of points
   * @param cost total cost of path
   * @param gradient of path at each X,Y pair from cost function derived analytically
   * @return if successful in computing values
   */
  // virtual bool Evaluate(
  //   const double * parameters,
  //   double * cost,
  //   double * gradient) const
  // {
  template <typename T>
  bool operator()(const T* const p, const T* const p_p1, const T* const p_m1, T* p_residual) const {
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi(p);
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi_p1(p_p1);
    Eigen::Map<const Eigen::Matrix<T, 2, 1> > xi_m1(p_m1);
    Eigen::Map<Eigen::Matrix<T, 4, 1> > residual(p_residual);
    residual.setZero();
    // cost[0] = 0.0;
    // double cost_raw = 0.0;
    // double grad_x_raw = 0.0;
    // double grad_y_raw = 0.0;
    // unsigned int mx, my;
    // bool valid_coords = true;
    // double costmap_cost = 0.0;

    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "EVALUATING %d points", (int)NumParameters()/2);

    // cache some computations between the residual and jacobian
    CurvatureComputations<T> curvature_params;
    // xi = Eigen::Vector2d(parameters[x_index], parameters[y_index]);
    // xi_p1 = Eigen::Vector2d(parameters[x_index + 2], parameters[y_index + 2]);
    // xi_m1 = Eigen::Vector2d(parameters[x_index - 2], parameters[y_index - 2]);

    // double smoothRes = 0, curvRes = 0, distRes = 0, costRes = 0;
    // compute cost
    addSmoothingResidual<T>(_params.smooth_weight, xi, xi_p1, xi_m1, residual[0]);//cost_raw);
    addCurvatureResidual<T>(_params.curvature_weight, xi, xi_p1, xi_m1, curvature_params, residual[1]);//cost_raw);
    addDistanceResidual<T>(_params.distance_weight, xi, _original_pos.template cast<T>(), residual[2]);//cost_raw);
    addCostResidual<T>(_costmap_weight, xi, xi_p1, xi_m1, residual[3]);//cost_raw);

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("optimizer_server"), "pt\n" << xi << "\nresidual\n" << residual << "\nw: " << _params.smooth_weight << ", " << _params.curvature_weight << ", " << _params.distance_weight << ", " << (_next_to_last_length_ratio < 0 ?  _params.dir_change_costmap_weight : _params.costmap_weight));

    // cost_raw += smoothRes + curvRes + distRes + costRes;

    // cost[i] = cost_raw;
    // cost[y_index] = cost_raw;

    // double smgx = 0, smgy = 0, cugx = 0, cugy = 0, digx = 0, digy = 0, cogx = 0, cogy = 0;
    // if (gradient != NULL) {
    //   // compute gradient
    //   grad_x_raw = 0.0;
    //   grad_y_raw = 0.0;

    //   addSmoothingJacobian(_params.smooth_weight, xi, xi_p1, xi_m1, smgx, smgy);//grad_x_raw, grad_y_raw);
    //   addCurvatureJacobian(
    //     _params.curvature_weight, xi, xi_p1, xi_m1, curvature_params,
    //     cugx, cugy);//grad_x_raw, grad_y_raw);
    //   addDistanceJacobian(
    //     _params.distance_weight, xi, _original_path->at(
    //       i), digx, digy);//grad_x_raw, grad_y_raw);

    //   if (valid_coords) {
    //     addCostJacobian(_params.costmap_weight, mx, my, costmap_cost, cogx, cogy);//grad_x_raw, grad_y_raw);
    //   }

    //   gradient[x_index] = smgx + cugx + digx + cogx;
    //   gradient[y_index] = smgy + cugy + digy + cogy;
    //   // gradient[x_index] = grad_x_raw;
    //   // gradient[y_index] = grad_y_raw;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "\npt: %lf %lf, smooth: %lf (%lf %lf), curv: %lf (%lf %lf), dist: %lf (%lf %lf), cost: %lf (%lf %lf), total: %lf (%lf %lf)",
    //             xi[0], xi[1], smoothRes, smgx, smgy, curvRes, cugx, cugy, distRes, digx, digy, costRes, cogx, cogy,
    //             smoothRes + curvRes + distRes + costRes, smgx + cugx + digx + cogx, smgy + cugy + digy + cogy);

    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "Total cost: %lf", cost_raw);
    // cost[0] = cost_raw;

    return true;
  }

protected:
  /**
   * @brief Cost function term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addSmoothingResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_p,
    const Eigen::Matrix<T, 2, 1> & pt_m,
    T & r) const
  {
    Eigen::Matrix<T, 2, 1> d_pt_p = pt_p - pt;
    Eigen::Matrix<T, 2, 1> d_pt_m = pt - pt_m;
    Eigen::Matrix<T, 2, 1> d_pt_diff = _next_to_last_length_ratio*d_pt_p - d_pt_m;
    r += (T)weight * d_pt_diff.dot(d_pt_diff);    // objective function value
  }

  /**
   * @brief Cost function derivative term for smooth paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addSmoothingJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & pt_m,
    double & j0,
    double & j1) const
  {
    j0 += weight *
      (-4 * pt_m[0] + 8 * pt[0] - 4 * pt_p[0]);   // xi x component of partial-derivative
    j1 += weight *
      (-4 * pt_m[1] + 8 * pt[1] - 4 * pt_p[1]);   // xi y component of partial-derivative
  }

  /**
   * @brief Cost function term for maximum curved paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct to cache computations for the jacobian to use
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCurvatureResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_p,
    const Eigen::Matrix<T, 2, 1> & pt_m,
    CurvatureComputations<T> & curvature_params,
    T & r) const
  {
    curvature_params.valid = true;
    curvature_params.delta_xi = Eigen::Matrix<T, 2, 1>(pt[0] - pt_m[0], pt[1] - pt_m[1]);
    curvature_params.delta_xi_p = Eigen::Matrix<T, 2, 1>(pt_p[0] - pt[0], pt_p[1] - pt[1]);
    if (_next_to_last_length_ratio < 0)
      curvature_params.delta_xi_p = -curvature_params.delta_xi_p;
    curvature_params.delta_xi_norm = curvature_params.delta_xi.norm();
    curvature_params.delta_xi_p_norm = curvature_params.delta_xi_p.norm();
    if (curvature_params.delta_xi_norm < EPSILON || curvature_params.delta_xi_p_norm < EPSILON ||
      ceres::IsNaN(curvature_params.delta_xi_p_norm) || ceres::IsNaN(curvature_params.delta_xi_norm) ||
      ceres::IsInfinite(curvature_params.delta_xi_p_norm) || ceres::IsInfinite(curvature_params.delta_xi_norm))
    {
      // ensure we have non-nan values returned
      curvature_params.valid = false;
      return;
    }

    const T & delta_xi_by_xi_p =
      curvature_params.delta_xi_norm * curvature_params.delta_xi_p_norm;
    T projection =
      curvature_params.delta_xi.dot(curvature_params.delta_xi_p) / delta_xi_by_xi_p;
    if (ceres::abs((T)1 - projection) < EPSILON || ceres::abs(projection + (T)1) < EPSILON) {
      projection = (T)1.0;
    }

    curvature_params.delta_phi_i = ceres::acos(projection);
    curvature_params.turning_rad = curvature_params.delta_phi_i / curvature_params.delta_xi_norm;

    curvature_params.ki_minus_kmax = curvature_params.turning_rad - _params.max_curvature;

    if (curvature_params.ki_minus_kmax <= EPSILON) {
      // Quadratic penalty need not apply
      curvature_params.valid = false;
      return;
    }

    r += (T)weight *
      curvature_params.ki_minus_kmax * curvature_params.ki_minus_kmax;  // objective function value
  }

  /**
   * @brief Cost function derivative term for maximum curvature paths
   * @param weight Weight to apply to function
   * @param pt Point Xi for evaluation
   * @param pt Point Xi+1 for calculating Xi's cost
   * @param pt Point Xi-1 for calculating Xi's cost
   * @param curvature_params A struct with cached values to speed up Jacobian computation
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addCurvatureJacobian(
    const double & weight,
    const Eigen::Vector2d & pt,
    const Eigen::Vector2d & pt_p,
    const Eigen::Vector2d & /*pt_m*/,
    CurvatureComputations<double> & curvature_params,
    double & j0,
    double & j1) const
  {
    if (!curvature_params.isValid()) {
      return;
    }

    const double & partial_delta_phi_i_wrt_cost_delta_phi_i =
      -1 / std::sqrt(1 - std::pow(std::cos(curvature_params.delta_phi_i), 2));
    // const Eigen::Vector2d ones = Eigen::Vector2d(1.0, 1.0);
    auto neg_pt_plus = -1 * pt_p;
    Eigen::Vector2d p1 = normalizedOrthogonalComplement(
      pt, neg_pt_plus, curvature_params.delta_xi_norm, curvature_params.delta_xi_p_norm);
    Eigen::Vector2d p2 = normalizedOrthogonalComplement(
      neg_pt_plus, pt, curvature_params.delta_xi_p_norm, curvature_params.delta_xi_norm);

    const double & u = 2 * curvature_params.ki_minus_kmax;
    const double & common_prefix =
      (1 / curvature_params.delta_xi_norm) * partial_delta_phi_i_wrt_cost_delta_phi_i;
    const double & common_suffix = curvature_params.delta_phi_i /
      (curvature_params.delta_xi_norm * curvature_params.delta_xi_norm);

    const Eigen::Vector2d & d_delta_xi_d_xi = curvature_params.delta_xi /
      curvature_params.delta_xi_norm;

    const Eigen::Vector2d jacobian = u *
      (common_prefix * (-p1 - p2) - (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_im1 = u *
      (common_prefix * p2 + (common_suffix * d_delta_xi_d_xi));
    const Eigen::Vector2d jacobian_ip1 = u * (common_prefix * p1);

    // Old formulation we may require again.
    // j0 += weight *
    //   (jacobian_im1[0] + 2 * jacobian[0] + jacobian_ip1[0]);
    // j1 += weight *
    //   (jacobian_im1[1] + 2 * jacobian[1] + jacobian_ip1[1]);

    j0 += weight * jacobian[0];  // xi x component of partial-derivative
    j1 += weight * jacobian[1];  // xi x component of partial-derivative
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addDistanceResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> & xi,
    const Eigen::Matrix<T, 2, 1> & xi_original,
    T & r) const
  {
    r += (T)weight * (xi - xi_original).dot(xi - xi_original);  // objective function value
  }

  /**
   * @brief Cost function derivative term for steering away changes in pose
   * @param weight Weight to apply to function
   * @param xi Point Xi for evaluation
   * @param xi_original original point Xi for evaluation
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addDistanceJacobian(
    const double & weight,
    const Eigen::Vector2d & xi,
    const Eigen::Vector2d & xi_original,
    double & j0,
    double & j1) const
  {
    j0 += weight * 2 * (xi[0] - xi_original[0]);  // xi y component of partial-derivative
    j1 += weight * 2 * (xi[1] - xi_original[1]);  // xi y component of partial-derivative
  }


  /**
   * @brief Cost function term for steering away from costs
   * @param weight Weight to apply to function
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param r Residual (cost) of term
   */
  template <typename T>
  inline void addCostResidual(
    const double & weight,
    const Eigen::Matrix<T, 2, 1> &pt,
    const Eigen::Matrix<T, 2, 1> &pt_p1,
    const Eigen::Matrix<T, 2, 1> &pt_m1,
    // const double & value,
    T & r) const
  {
    // if (value == FREE) {
    //   return;
    // }
    // _costmap->worldToMap(xi[0], xi[1], mx, my);
    double origx = _costmap->getOriginX();
    double origy = _costmap->getOriginY();
    double res = _costmap->getResolution();
    int sizex = _costmap->getSizeInCellsX();
    int sizey = _costmap->getSizeInCellsY();

    if (!_params.cost_check_points.empty()) {
      Eigen::Matrix<T, 2, 1> dir = tangentDir(pt_m1, pt, pt_p1, _next_to_last_length_ratio < 0 ? -1 : 1);
      dir.normalize();
      if (((pt_p1 - pt).dot(dir) < (T)0) != _reversing)
        dir = -dir;
      Eigen::Matrix<T, 3, 3> transform;
      transform << dir[0], -dir[1], pt[0],
                  dir[1], dir[0], pt[1],
                  (T)0, (T)0, (T)1;
      // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "pt: %lf %lf, dir: %lf %lf", pt[0], pt[1], dir[0], dir[1]);
      for (size_t i = 0; i < _params.cost_check_points.size(); i += 3) {
        Eigen::Matrix<T, 3, 1> ccpt((T)_params.cost_check_points[i], (T)_params.cost_check_points[i+1], (T)1);
        auto ccptWorld = transform*ccpt;
        T interpx = (ccptWorld[0] - (T)origx) / (T)res - (T)0.5;
        T interpy = (ccptWorld[1] - (T)origy) / (T)res - (T)0.5;
        T value;
        interpolateCostmap->Evaluate(interpy, interpx, &value);

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("optimizer_server"), "trans\n" << transform << "\nccpt\n" << ccptWorld << "\nw: " << _params.cost_check_points[i+2] << ", value: " << value);
        r += (T)weight * (T)_params.cost_check_points[i+2] * value * value;
      }
    }
    else {
      T interpx = (pt[0] - (T)origx) / (T)res - (T)0.5;
      T interpy = (pt[1] - (T)origy) / (T)res - (T)0.5;
      T value;
      interpolateCostmap->Evaluate(interpy, interpx, &value);
      // int m00x = static_cast<int>(interpx);
      // int m00y = static_cast<int>(interpy);
      // if (m00x < 0 || m00y < 0)
      //   return;
      // interpx -= m00x;
      // interpy -= m00y;
      // int m11x = m00x + 1;
      // int m11y = m00y + 1;
      // if (m11x >= sizex || m11y >= sizey)
      //   return;
      // T value = (  (T)_costmap->getCost(m00x, m00y)*((T)1 - interpy)
      //                 + (T)_costmap->getCost(m00x, m11y)*interpy      )*((T)1 - interpx)
      //              + (  (T)_costmap->getCost(m11x, m00y)*((T)1 - interpy)
      //                 + (T)_costmap->getCost(m11x, m11y)*interpy      )*interpx;
      
      r += (T)weight * value * value;  // objective function value
      // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "pt: %lf %lf, val: %lf, residual: %lf", pt[0], pt[1], value, weight*value*value);
    }
  }

  /**
   * @brief Cost function derivative term for steering away from costs
   * @param weight Weight to apply to function
   * @param mx Point Xi's x coordinate in map frame
   * @param mx Point Xi's y coordinate in map frame
   * @param value Point Xi's cost'
   * @param params computed values to reduce overhead
   * @param j0 Gradient of X term
   * @param j1 Gradient of Y term
   */
  inline void addCostJacobian(
    const double & weight,
    const unsigned int & mx,
    const unsigned int & my,
    const double & value,
    double & j0,
    double & j1) const
  {
    // if (value == FREE) {
    //   return;
    // }

    const Eigen::Vector2d grad = getCostmapGradient(mx, my);
    const double common_prefix = 2.0 * weight * value;
    // const double common_prefix = -2.0 * _params.costmap_factor * weight * value * value;

    j0 += common_prefix * grad[0];  // xi x component of partial-derivative
    j1 += common_prefix * grad[1];  // xi y component of partial-derivative
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "grad: %lf %lf, adj: %lf %lf", grad[0], grad[1], common_prefix * grad[0], common_prefix * grad[1]);
  }

  /**
   * @brief Computing the gradient of the costmap using
   * the 2 point numerical differentiation method
   * @param mx Point Xi's x coordinate in map frame
   * @param mx Point Xi's y coordinate in map frame
   * @param params Params reference to store gradients
   */
  inline Eigen::Vector2d getCostmapGradient(
    const unsigned int mx,
    const unsigned int my) const
  {
    // find unit vector that describes that direction
    // via 7 point taylor series approximation for gradient at Xi
    Eigen::Vector2d gradient;

    double l_1 = 0.0;
    double l_2 = 0.0;
    double l_3 = 0.0;
    double r_1 = 0.0;
    double r_2 = 0.0;
    double r_3 = 0.0;

    if (mx < _costmap->getSizeInCellsX()) {
      r_1 = static_cast<double>(_costmap->getCost(mx + 1, my));
    }
    if (mx + 1 < _costmap->getSizeInCellsX()) {
      r_2 = static_cast<double>(_costmap->getCost(mx + 2, my));
    }
    if (mx + 2 < _costmap->getSizeInCellsX()) {
      r_3 = static_cast<double>(_costmap->getCost(mx + 3, my));
    }

    if (mx > 0) {
      l_1 = static_cast<double>(_costmap->getCost(mx - 1, my));
    }
    if (mx - 1 > 0) {
      l_2 = static_cast<double>(_costmap->getCost(mx - 2, my));
    }
    if (mx - 2 > 0) {
      l_3 = static_cast<double>(_costmap->getCost(mx - 3, my));
    }

    gradient[0] = (45 * r_1 - 9 * r_2 + r_3 - 45 * l_1 + 9 * l_2 - l_3) / 60;
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "X: g: %lf r1: %lf, r2: %lf, r3: %lf, l1: %lf, l2: %lf, l3: %lf", gradient[0], r_1, r_2, r_3, l_1, l_2, l_3);

    if (my < _costmap->getSizeInCellsY()) {
      r_1 = static_cast<double>(_costmap->getCost(mx, my + 1));
    }
    if (my + 1 < _costmap->getSizeInCellsY()) {
      r_2 = static_cast<double>(_costmap->getCost(mx, my + 2));
    }
    if (my + 2 < _costmap->getSizeInCellsY()) {
      r_3 = static_cast<double>(_costmap->getCost(mx, my + 3));
    }

    if (my > 0) {
      l_1 = static_cast<double>(_costmap->getCost(mx, my - 1));
    }
    if (my - 1 > 0) {
      l_2 = static_cast<double>(_costmap->getCost(mx, my - 2));
    }
    if (my - 2 > 0) {
      l_3 = static_cast<double>(_costmap->getCost(mx, my - 3));
    }

    gradient[1] = (45 * r_1 - 9 * r_2 + r_3 - 45 * l_1 + 9 * l_2 - l_3) / 60;
    // RCLCPP_INFO(rclcpp::get_logger("optimizer_server"), "Y: g: %lf r1: %lf, r2: %lf, r3: %lf, l1: %lf, l2: %lf, l3: %lf", gradient[1], r_1, r_2, r_3, l_1, l_2, l_3);

    // gradient.normalize();
    return gradient;
  }

  /**
   * @brief Computing the normalized orthogonal component of 2 vectors
   * @param a Vector
   * @param b Vector
   * @param norm a Vector's norm
   * @param norm b Vector's norm
   * @return Normalized vector of orthogonal components
   */
  inline Eigen::Vector2d normalizedOrthogonalComplement(
    const Eigen::Vector2d & a,
    const Eigen::Vector2d & b,
    const double & a_norm,
    const double & b_norm) const
  {
    return (a - (a.dot(b) * b / b.squaredNorm())) / (a_norm * b_norm);
  }

  const Eigen::Vector2d &_original_pos;
  nav2_costmap_2d::Costmap2D * _costmap{nullptr};
  SmootherParams _params;
  double _costmap_weight;
  double _next_to_last_length_ratio;
  bool _reversing;
  std::unique_ptr<ceres::Grid2D<u_char>> costmapGrid;
  std::unique_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> interpolateCostmap;
};

}  // namespace nav2_smac_planner

#endif  // RTV_NAV2_SMAC_PLANNER__SMOOTHER_COST_FUNCTION_HPP_
