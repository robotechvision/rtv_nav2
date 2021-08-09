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

// #define CSC_PURSUIT_DEBUG

#include <algorithm>
#include <string>
#include <memory>
#include <utility>

#include "rtv_nav2_csc_pursuit_controller/csc_pursuit_controller.hpp"
#include "rtv_nav2_csc_pursuit_controller/shape_iter.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include <opencv2/imgproc.hpp>

#ifdef CSC_PURSUIT_DEBUG
#include <opencv2/highgui.hpp>
#endif

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace rtv_nav2_csc_pursuit_controller
{

inline double normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;
  
  double multiplier = std::floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

inline double norm2(const cv::Point2d &p) {
    return p.x*p.x + p.y*p.y;
}

template<typename T>
inline int sign(T a) {
  return a == 0 ? 0 : a > 0 ? 1 : -1;
}

template<typename T>
inline T sqr(T a) {
    return a*a;
}

inline geometry_msgs::msg::Point msgFromCv(cv::Point2d p) {
  geometry_msgs::msg::Point r;
  r.x = p.x;
  r.y = p.y;
  return r;
}

inline geometry_msgs::msg::PoseStamped msgFromCv(cv::Point2d p, double a, const std_msgs::msg::Header &header) {
    geometry_msgs::msg::PoseStamped r;
    r.header = header;
    r.pose.position.x = p.x;
    r.pose.position.y = p.y;
    r.pose.orientation.w = cos(a/2);
    r.pose.orientation.z = sin(a/2);
    return r;
}

inline double segmentPointDistance(const geometry_msgs::msg::Point &segp1, const geometry_msgs::msg::Point &segp2,
                                   const geometry_msgs::msg::Point &p, double *s) {
    double t;
    geometry_msgs::msg::Point d;
    d.x = segp2.x - segp1.x;
    d.y = segp2.y - segp1.y;
    if (std::abs(d.x) > std::abs(d.y)) {
        t = -(p.y - segp1.y + (segp1.x - p.x)*d.y/d.x)/(d.x + d.y*d.y/d.x);
        *s = (p.x - segp1.x - t*d.y)/d.x;
    }
    else {
        t = +(p.x - segp1.x + (segp1.y - p.y)*d.x/d.y)/(d.y + d.x*d.x/d.y);
        *s = (p.y - segp1.y + t*d.x)/d.y;
    }
    if (*s < 0) {
        *s = 0;
        return sqrt(sqr(p.x - segp1.x) + sqr(p.y - segp1.y));
    }
    else if (*s > 1) {
        *s = 1;
        return sqrt(sqr(p.x - segp2.x) + sqr(p.y - segp2.y));
    }
    return std::abs(t)*sqrt(d.x*d.x + d.y*d.y);
}

inline double arcPointDistance(const geometry_msgs::msg::Point &c, double r, double a1, double a2, const geometry_msgs::msg::Point &p) {
  // we assume a1 is normalized
  // // put a1 into interval <-pi; pi)
  // double a1c = floor((a1 + M_PI)/(M_PI*2))*M_PI*2;
  // a1 -= a1c;
  // a2 -= a1c;

  double a = atan2(p.y - c.y, p.x - c.x);
  if (a > a1 && a2 <= a1)
    a -= M_PI*2;
  else if (a < a1 && a2 >= a1)
    a += M_PI*2;
  
  if (a >= a1 && a <= a2)
    return std::abs(std::hypot(p.x - c.x, p.y - c.y) - r);
  else
    return sqrt(std::min(  sqr(p.x - (c.x + cos(a1)*r)) + sqr(p.y - (c.y + sin(a1)*r))
                         , sqr(p.x - (c.x + cos(a2)*r)) + sqr(p.y - (c.y + sin(a2)*r))));
}

void MovementsSequence::fromPosesWithCircles(cv::Point2d s, cv::Point2d ns, cv::Point2d t, cv::Point2d nt,
                                  double r, bool inner, bool reverseDir, std::vector<MovementsSequence> &output) {
    //inputs
    cv::Point2d ms = s + ns*r;
    cv::Point2d mt = t + nt*r;

    if (inner && norm2(mt - ms) < r*r*4) {
        //shrink circles
        cv::Point2d nd = nt - ns;
        cv::Point2d d = t - s;
        double a = norm2(nd) - 4;
        double b = 2*nd.ddot(d);
        double c = norm2(d);
        double disc = b*b - 4*a*c;
        assert(disc >= 0);
        disc = sqrt(disc);
        double r1 = (-b + disc)/(2*a);
        double r2 = (-b - disc)/(2*a);
        r = std::max(0.001, (r1 >= 0 && r1 < r2) ? r1 : r2); //border for computation safety
        ms = s + ns*r;
        mt = t + nt*r;
    }

    //compute tangentials
    double da;
    double ds;
    double tsign;
    auto dm = mt - ms;
    double adm = atan2(dm.y, dm.x);
    if (inner) {
        double dth = cv::norm(dm)/2; //thales circle diameter
        double x = r*r/dth;
        da = acos(std::min(1.0, x/r)); //x/r can be a bit higher than 1 because of float computation errors
        double sqry = std::max(0.0, r*r - x*x);
        ds = 2*sqrt(sqry + sqr(dth-x));
        tsign = -1;
    }
    else {
        ds = cv::norm(dm);
        da = M_PI/2;
        tsign = 1;
    }
    double a1s = normalize_theta(adm - da);
    double a1t = normalize_theta(adm + M_PI + da*tsign);
    double a2s = normalize_theta(adm + da);
    double a2t = normalize_theta(adm + M_PI - da*tsign);

    double as = atan2(-ns.y, -ns.x);
    double at = atan2(-nt.y, -nt.x);

    double sdir = reverseDir ? -1 : 1;
    double das = a1s-as;
    double dat = at-a1t;
    output.emplace_back(das, 1, dat, 1, tsign, r, ds, 1, sdir);        
    output.emplace_back(das, 1, dat, -1, tsign, r, ds, 1, sdir);        
    output.emplace_back(das, -1, dat, 1, tsign, r, ds, 1, sdir);        
    output.emplace_back(das, -1, dat, -1, tsign, r, ds, 1, sdir);        
    das = a2s-as;
    dat = at-a2t;
    output.emplace_back(das, 1, dat, 1, tsign, r, ds, -1, sdir);        
    output.emplace_back(das, 1, dat, -1, tsign, r, ds, -1, sdir);        
    output.emplace_back(das, -1, dat, 1, tsign, r, ds, -1, sdir);        
    output.emplace_back(das, -1, dat, -1, tsign, r, ds, -1, sdir);        
}

double MovementsSequence::length() {
    double result = 0;
    for (auto &mt : movements)
        result += std::abs(mt.l);
    return result;
}

int MovementsSequence::directionChangesCount(int lastDir) {
    int result = 0;
    for (auto &mt : movements) {
        if ((mt.l > 0) != (lastDir > 0)) {
            if (lastDir != 0 && mt.l != 0)
                result++;
            lastDir = mt.l == 0 ? 0 : (mt.l > 0 ? 1 : -1);
        }
    }
    return result;
}

double MovementsSequence::minRotRadius() {
    double result = std::numeric_limits<double>::infinity();
    for (auto &mt : movements)
        if (std::abs(mt.r) < result)
            result = std::abs(mt.r);
    return result;
}

double MovementsSequence::trajectoryResidualsPositional(const std::vector<geometry_msgs::msg::PoseStamped> &traj, double &trajLen, rclcpp::Logger *logger) {
  //TODO: orientationRes
  trajLen = 0;
  double positionRes = 0;
  for (int i = 0; i < traj.size(); i++) {
    auto &pose = traj[i].pose;
    
    if (i != 0)
      trajLen += std::hypot(pose.position.x - traj[i-1].pose.position.x, pose.position.y - traj[i-1].pose.position.y);

    cv::Point2d s(0, 0);
    double a = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (auto &mt : movements) {
      double dist;
      if (!std::isfinite(mt.r)) {
        auto segp1 = msgFromCv(s);
        s += cv::Point2d(cos(a), sin(a))*mt.l;
        double dummyS;
        dist = segmentPointDistance(segp1, msgFromCv(s), pose.position, &dummyS);
        if (logger)
          RCLCPP_INFO((*logger), "straight (%lf) %lf %lf -> %lf %lf, dist: %lf", mt.l, segp1.x, segp1.y, s.x, s.y, dist);
      }
      else {
        double da = mt.l/mt.r;
        cv::Point2d mid = s + cv::Point2d(-sin(a), cos(a))*mt.r;
        double tangentialToCentrifugal = mt.r > 0 ? -M_PI/2 : M_PI/2;
        dist = arcPointDistance(msgFromCv(mid), std::abs(mt.r), a + tangentialToCentrifugal, a + da + tangentialToCentrifugal, pose.position);
        if (logger)
          RCLCPP_INFO((*logger), "arc (%lf, %lf) center: %lf %lf, a1: %lf, a2: %lf, ap: %lf, dist: %lf", mt.l, mt.r, mid.x, mid.y, a, a+da,
                atan2(pose.position.y - mid.y, pose.position.x - mid.x), dist);
        a = normalize_theta(a + da);
        s = mid - cv::Point2d(-sin(a), cos(a))*mt.r;
      }
      if (dist < bestDist)
        bestDist = dist;
    }
    if (logger)
          RCLCPP_INFO((*logger), "pose: %lf %lf, dist: %lf", pose.position.x, pose.position.y, bestDist);
    positionRes += bestDist;
  }
  if (logger)
          RCLCPP_INFO((*logger), "len: %lf, res: %lf", trajLen, positionRes*trajLen/traj.size());
  return traj.size() < 1 ? 0 : positionRes*trajLen/traj.size();
}

double MovementsSequence::trajectoryResidualsAngular(const std::vector<geometry_msgs::msg::PoseStamped> &traj, double trajLen) {
  double len = length();
  double angleRes = 0;
  int mti = 0;
  double mts = 0;
  double a = 0;
  // printf("len: %lf, trajLen: %lf\n", len, trajLen);
  for (int i = 1; i < traj.size(); i++) {
    auto &pose = traj[i].pose;
    
    mts += std::hypot(pose.position.x - traj[i-1].pose.position.x, pose.position.y - traj[i-1].pose.position.y)*len/trajLen;

    while (mti < movements.size() - 1 && mts > std::abs(movements[mti].l)) {
      auto &mt = movements[mti];
      if (std::isfinite(mt.r)) {
        double da = mt.l/mt.r;
        a = normalize_theta(a + da);
      }
      mti++;
      mts -= std::abs(mt.l);
    }

    auto &mt = movements[mti];
    double da = mts*sign(mt.l)/mt.r;
    tf2::Quaternion q;
    tf2::convert(pose.orientation, q);
    double trajPointAngle = q.getAngle()*sign(q.getZ());
    angleRes += std::abs(normalize_theta(a + da - trajPointAngle));

    // printf("mti: %d, mts: %lf, l: %lf, r: %lf, a: %lf, a+da: %lf, ta: %lf, res: %lf\n", mti, mts, mt.l, mt.r, a, a+da, trajPointAngle, angleRes);
  }
  // printf("res: %lf\n", traj.size() < 2 ? 0.0 : angleRes*trajLen/(traj.size()-1));
  return traj.size() < 2 ? 0.0 : angleRes*trajLen/(traj.size()-1);
}

double MovementsSequence::avgCost(double maxPosDiff, double distPerRot, const std::vector<tf2::Vector3> &footprint,
                                  const tf2::Transform& pose, const nav2_costmap_2d::Costmap2D &costmap, bool visualize, rclcpp::Logger *logger) {
  int mti = 0;
  double mts = 0;
  cv::Point2d s(0, 0);
  double a = 0;

  double totalCost = 0;
  int checksCnt = 0;
  cv::Mat cmap(costmap.getSizeInCellsY(), costmap.getSizeInCellsX(), CV_8UC1, costmap.getCharMap());
  cv::Mat cmapViz;
  if (visualize)
    cmap.copyTo(cmapViz);
  while (true) {
    double diffRemaining = maxPosDiff;
    double rotMultiplier = 1 + distPerRot/std::abs(movements[mti].r);
    while (mti < movements.size() && mts + diffRemaining/rotMultiplier > std::abs(movements[mti].l)) {
      auto &mt = movements[mti];
      if (!std::isfinite(mt.r))
        s += cv::Point2d(cos(a), sin(a))*mt.l;
      else {
        double da = mt.l/mt.r;
        cv::Point2d mid = s + cv::Point2d(-sin(a), cos(a))*mt.r;
        a = normalize_theta(a + da);
        s = mid - cv::Point2d(-sin(a), cos(a))*mt.r;
      }

      diffRemaining -= (std::abs(movements[mti].l) - mts)*rotMultiplier;
      mts = 0;
      mti++;
      if (mti < movements.size())
        rotMultiplier = 1 + distPerRot/std::abs(movements[mti].r);
    }
    if (mti == movements.size())
      break;
    mts += diffRemaining/rotMultiplier;
    // if (logger)
    //   RCLCPP_INFO((*logger), "mti: %d, mts: %lf, s: %lf %lf\n", mti, mts, s.x, s.y);
    
    checksCnt++;
    auto &mt = movements[mti];
    double da = mts*sign(mt.l)/mt.r;

    cv::Point2d ss;
    if (!std::isfinite(mt.r))
      ss = s + cv::Point2d(cos(a), sin(a))*mts*sign(mt.l);
    else {
      cv::Point2d mid = s + cv::Point2d(-sin(a), cos(a))*mt.r;
      ss = mid - cv::Point2d(-sin(a + da), cos(a + da))*mt.r;
    }
    tf2::Transform checkPose = pose*tf2::Transform(tf2::Quaternion(tf2::Vector3(0, 0, 1), a + da), tf2::Vector3(ss.x, ss.y, 0));
    std::vector<cv::Point2l> poly;
    poly.reserve(footprint.size());
    double res = costmap.getResolution();
    double origX = costmap.getOriginX();
    double origY = costmap.getOriginY();
    for (auto &fp : footprint) {
      auto fpt = checkPose(fp);
      poly.emplace_back((fpt.x() - origX)/res, (fpt.y() - origY)/res);
    }

    if (visualize) {
      std::vector<std::vector<cv::Point2i>> polyViz(1);
      for (auto &p : poly)
        polyViz[0].push_back(p);
      cv::polylines(cmapViz, polyViz, true, cv::Scalar(255));
    }

    double footprintCost = 0;
    int pixelsCheckedCnt = 0;
    cv::iter::iterateFillPoly(cmap, poly.data(), poly.size(), [&footprintCost, &pixelsCheckedCnt](uint32_t cnt, uchar *data) {
      pixelsCheckedCnt += cnt;
      for (uint32_t i = 0; i < cnt; i++, data++) {
          if (*data == nav2_costmap_2d::LETHAL_OBSTACLE)
            footprintCost = std::numeric_limits<double>::infinity();
          else if (*data != nav2_costmap_2d::NO_INFORMATION)
            footprintCost += (*data)/(double)nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
      }
    }, 8);
    totalCost += footprintCost/pixelsCheckedCnt;
  }
#ifdef CSC_PURSUIT_DEBUG
  if (visualize) {
    cv::imshow("cost polys", cmapViz);
    cv::waitKey(30);
  }
#endif
  return totalCost/checksCnt;
}

void CscPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".base_footprint_frame", rclcpp::ParameterValue("base_footprint"));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_interval", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".repeat_interval", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_no_data_time", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_linear", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_linear_decrease_per_rotational_velocity", rclcpp::ParameterValue(0.6/2)); //wheel distance/2
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_rotational", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_velocity_rotational_decrease_per_radius_decrease", rclcpp::ParameterValue(0.75));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_max_velocity_rotational", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotation_radius", rclcpp::ParameterValue(0.7));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_target_dist_to_finish", rclcpp::ParameterValue(0.5));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_carrot_dist", rclcpp::ParameterValue(0.7));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rrads_per_direction_change_penalty", rclcpp::ParameterValue(4.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rrads_per_rrad_change_penalty", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_movement_duration", rclcpp::ParameterValue(0.2)); // higher value makes trajectory smoother but less accurate
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_traj_poses_skip", rclcpp::ParameterValue(100000));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_traj_error_positional_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_traj_error_longitudinal_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_traj_error_angular_penalty", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".control_traj_error_cost_penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_check_step", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_check_meters_per_radian", rclcpp::ParameterValue(0.35));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_distance_multiplier", rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".should_control", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".blind_start", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".base_footprint_frame", baseFootprintFrame);
  node->get_parameter(plugin_name_ + ".control_interval", controlInterval);
  // node->get_parameter(plugin_name_ + ".repeat_interval", repeatInterval);
  node->get_parameter(plugin_name_ + ".max_no_data_time", maxNoDataTime);

  node->get_parameter(plugin_name_ + ".max_velocity_linear", maxVelLin);
  maxVelLinBase = maxVelLin;
  node->get_parameter(plugin_name_ + ".max_velocity_linear_decrease_per_rotational_velocity", maxVelLinDecPerVelRot);
  node->get_parameter(plugin_name_ + ".max_velocity_rotational", maxVelRot);
  node->get_parameter(plugin_name_ + ".max_velocity_rotational_decrease_per_radius_decrease", maxVelRotDecPerRRadDec);
  node->get_parameter(plugin_name_ + ".min_max_velocity_rotational", minMaxVelRot);
  node->get_parameter(plugin_name_ + ".rotation_radius", rrad);
  node->get_parameter(plugin_name_ + ".min_target_dist_to_finish", minTargetDistToFinish);
  node->get_parameter(plugin_name_ + ".min_carrot_dist", minCarrotDist);

  node->get_parameter(plugin_name_ + ".rrads_per_direction_change_penalty", rradsPerDirChange);
  node->get_parameter(plugin_name_ + ".rrads_per_rrad_change_penalty", rradsPerRRadChange);
  node->get_parameter(plugin_name_ + ".min_movement_duration", minMvmtDur);
  node->get_parameter(plugin_name_ + ".max_traj_poses_skip", maxTrajPosesSkip);
  node->get_parameter(plugin_name_ + ".control_traj_error_positional_penalty", controlTrajErrPosPenalty);
  node->get_parameter(plugin_name_ + ".control_traj_error_longitudinal_penalty", controlTrajErrLenPenalty);
  node->get_parameter(plugin_name_ + ".control_traj_error_angular_penalty", controlTrajErrAnglePenalty);
  node->get_parameter(plugin_name_ + ".control_traj_error_cost_penalty", controlTrajErrCostPenalty);
  node->get_parameter(plugin_name_ + ".cost_check_step", costCheckStep);
  node->get_parameter(plugin_name_ + ".cost_check_meters_per_radian", costCheckMetersPerRadian);

  node->get_parameter(plugin_name_ + ".angular_distance_multiplier", angularDistMultiplier);

  node->get_parameter(plugin_name_ + ".should_control", shouldControl);
  node->get_parameter(plugin_name_ + ".blind_start", blindStart);

  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter("controller_frequency", control_frequency);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  repeatInterval = 1.0 / control_frequency;

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  residuals_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("residuals", 1);
  control_start_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("control_start", 1);
}

void CscPursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " rtv_nav2_csc_pursuit_controller::CscPursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  local_plan_pub_.reset();
  residuals_pub_.reset();
  control_start_pub_.reset();
}

void CscPursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "rtv_nav2_csc_pursuit_controller::CscPursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  local_plan_pub_->on_activate();
  residuals_pub_->on_activate();
  control_start_pub_->on_activate();
}

void CscPursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "rtv_nav2_csc_pursuit_controller::CscPursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  residuals_pub_->on_deactivate();
  control_start_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped CscPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  if (trajChanged) {
    trajChanged = false;
    if (shouldControl || !blindStart)
        controlTimerCallback(pose);
    else {
        std::vector<Movement> mvmts;
        planMovements(mvmts, 0, 
          rclcpp::Time(pose.header.stamp) - rclcpp::Time(currentCmdVel.header.stamp) > rclcpp::Duration::from_seconds(0.2)
            ? 0
            : sign(currentCmdVel.twist.linear.x));

        visualizeMovements(mvmts, pose);
    }
  }
  repeatTimerCallback(rclcpp::Time(pose.header.stamp));

  if (shouldControl) {
    double lastControlDurSinceTraj = (controlTimerStamp - rclcpp::Time(traj.header.stamp)).seconds();
    double durSinceTraj = (rclcpp::Time(pose.header.stamp) - rclcpp::Time(traj.header.stamp)).seconds();
    if ((int)(durSinceTraj/controlInterval) > (int)(lastControlDurSinceTraj/controlInterval))
      controlTimerCallback(pose);
  }

  // populate and return message
  currentCmdVel.header = pose.header;
  // currentCmdVel.twist = geometry_msgs::msg::Twist();
  return currentCmdVel;
}

void CscPursuitController::planMovements(std::vector<Movement> &mvmts, int starti, int lastDir) {
    for (int i = starti; i < traj.poses.size()-1; i++) {
        tf2::Transform ps, pt; // TODO: optimize
        tf2::convert(traj.poses[i].pose, ps);
        tf2::convert(traj.poses[i+1].pose, pt);

        auto &pose = traj.poses[i+1].pose;
        tf2::Vector3 mt(pose.position.x - traj.poses[i].pose.position.x, pose.position.y - traj.poses[i].pose.position.y, 0);
        tf2::Quaternion q;
        tf2::convert(pose.orientation, q);
        double angle = q.getAngle()*sign(q.getZ());
        tf2::Vector3 ori(cos(angle), sin(angle), 0);
        mt.normalize();
        int newDir = ori.dot(mt) >= 0 ? 1 : -1;
        int desiredDirChanges = lastDir != 0 && newDir != lastDir ? 1 : 0;
        // RCLCPP_INFO(logger_, "mt: %lf %lf (%lf %lf -> %lf %lf), ori: %lf %lf (%lf), dir: %d, change: %d",
        //             mt.x(), mt.y(), traj.poses[i].pose.position.x, traj.poses[i].pose.position.y, pose.position.x, pose.position.y,
        //             ori.x(), ori.y(), angle/M_PI*180, newDir, desiredDirChanges);

        auto seq = movementsBetweenPoses(ps, pt, lastDir, desiredDirChanges);
        // printf("Inserting %d movements\n", (int)seq.movements.size());
        mvmts.insert(mvmts.end(), seq.movements.begin(), seq.movements.end());
        lastDir = sign(mvmts.back().l);
    }
    joinShortMovements(mvmts);
    movements = mvmts;
    currentMovement = -1;
    currentMovementEnd = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void CscPursuitController::joinShortMovements(std::vector<Movement> &mvmts) {
    for (int i = 0; i < mvmts.size(); i++) {
        if (mvmts.size() == 1)
            break;
        if (i > 0)
            break; //only first movement
        auto &mt = mvmts[i];
        double da = mt.l/mt.r;
        double dur;

        double mvr = maxVelRot;
        double r = std::abs(mt.r);
        if (r < rrad)
            mvr = std::max(minMaxVelRot, maxVelRot - maxVelRotDecPerRRadDec*(rrad-r)/rrad);

        if (std::abs(mt.l) < 0.001) {
            dur = std::abs(da)/mvr;
            // printf("Movement %d (l: %lf, r: %lf) -> omega: %lf -> dur: %lf\n", i, mt.l, mt.r, mvr, dur);
        }
        else {
            double dal = std::abs(da/mt.l);
            double v = maxVelLin/(1 + maxVelLinDecPerVelRot*dal);
            double om = v*dal;
            if (om > mvr) {
                om = mvr;
                v = om/dal;
            }
            dur = std::abs(mt.l)/v;
            // printf("Movement %d (l: %lf, r: %lf) -> v: %lf, omega: %lf (/%lf) -> dur: %lf\n", i, mt.l, mt.r, v, om, mvr, dur);
        }
        if (dur < minMvmtDur) {
            int ci = 0;
            if (i == 0) {
                if ((mvmts[1].l < 0) == (mt.l < 0))
                    ci = 1;
            }
            else if (i == mvmts.size()-1) {
                if ((mvmts[i-1].l < 0) == (mt.l < 0))
                    ci = -1;
            }
            else if ((mvmts[i-1].l < 0) == (mt.l < 0)
                      && (std::abs(mvmts[i-1].r) > std::abs(mvmts[i+1].r)
                          || std::abs(mvmts[i-1].r) == std::abs(mvmts[i+1].r) && (mvmts[i-1].r < 0) == (mt.r < 0)))
                ci = -1;
            else if ((mvmts[i+1].l < 0) == (mt.l < 0))
                ci = 1;
            
            if (ci == 0)
                continue;
            
            auto &mtc = mvmts[i+ci];
            double l = std::min(minMvmtDur*maxVelLin, std::abs(mtc.l));
            double rat = l/std::abs(mtc.l);
            double al = rat*mtc.l;
            double aa = rat*(mtc.l/mtc.r);

            // printf("Joining movement %d (l: %lf; r: %lf) with %d (l: %lf; r: %lf; rat: %lf)", i, mt.l, mt.r, i+ci, mtc.l, mtc.r, rat);
            mt.l += al;
            mt.r = mt.l/(da+aa);
            // printf(" -> (l: %lf; r: %lf)\n", mt.l, mt.r);

            if (rat == 1) {
                // printf("erasing empty movement\n");
                mvmts.erase(mvmts.begin()+i+ci);
                if (ci == -1)
                    i--;
            }
            else
                mtc.l -= al;
            
            i--;
        }
    }
    // printf("JOINING FINISHED\n");
}

void CscPursuitController::repeatTimerCallback(const rclcpp::Time &now) {
    if ((now - trajTime).seconds() > maxNoDataTime) {
        currentCmdVel = geometry_msgs::msg::TwistStamped();
        return;
    }

    if (now + rclcpp::Duration::from_seconds(repeatInterval/2) > currentMovementEnd) {
        currentMovement++;
        if (currentMovement >= movements.size()) {
            currentCmdVel = geometry_msgs::msg::TwistStamped();
            return;
        }
        auto &mt = movements[currentMovement];
        double da = mt.l/mt.r;
        double dur;

        double mvr = maxVelRot;
        double r = std::abs(mt.r);
        if (r < rrad)
            mvr = std::max(minMaxVelRot, maxVelRot - maxVelRotDecPerRRadDec*(rrad-r)/rrad);

        if (std::abs(mt.l) < 0.001) {
            dur = std::abs(da)/mvr;
            // printf("Movement %d (l: %lf, r: %lf) -> omega: %lf -> dur: %lf\n", currentMovement, mt.l, mt.r, mvr, dur);
        }
        else {
            double dal = std::abs(da/mt.l);
            double v = maxVelLin/(1 + maxVelLinDecPerVelRot*dal);
            double om = v*dal;
            if (om > mvr) {
                om = mvr;
                v = om/dal;
            }
            dur = std::abs(mt.l)/v;
            // printf("Movement %d (l: %lf, r: %lf) -> v: %lf, omega: %lf -> dur: %lf\n", currentMovement, mt.l, mt.r, v, om, dur);
        }
        currentMovementEnd = now + rclcpp::Duration::from_seconds(dur);
        currentCmdVel.twist.linear.x = mt.l/dur;
        currentCmdVel.twist.angular.z = da/dur;
    }
}

void CscPursuitController::visualizeMovements(const std::vector<Movement> &mvmts, const geometry_msgs::msg::PoseStamped &ps) {
    cv::Point2d s(0, 0);
    s.x = ps.pose.position.x;
    s.y = ps.pose.position.y;
    tf2::Transform pstf;
    tf2::convert(ps.pose, pstf);
    double a = atan2(pstf.getBasis().getColumn(0).y(), pstf.getBasis().getColumn(0).x());

    nav_msgs::msg::Path path;
    path.header = ps.header;
    path.poses.push_back(msgFromCv(s, a, ps.header));
    for (auto &mt : mvmts) {
        double da = mt.l/mt.r;
        double dur;

        double mvr = maxVelRot;
        double r = std::abs(mt.r);
        if (r < rrad)
            mvr = std::max(minMaxVelRot, maxVelRot - maxVelRotDecPerRRadDec*(rrad-r)/rrad);

        if (std::abs(mt.l) < 0.001) {
            dur = std::abs(da)/mvr;
            // printf("Movement %d (l: %lf, r: %lf) -> omega: %lf -> dur: %lf\n", currentMovement, mt.l, mt.r, mvr, dur);
        }
        else {
            double dal = std::abs(da/mt.l);
            double v = maxVelLin/(1 + maxVelLinDecPerVelRot*dal);
            double om = v*dal;
            if (om > mvr) {
                om = mvr;
                v = om/dal;
            }
            dur = std::abs(mt.l)/v;
            // printf("Movement %d (l: %lf, r: %lf) -> v: %lf, omega: %lf -> dur: %lf\n", currentMovement, mt.l, mt.r, v, om, dur);
        }

        if (!std::isfinite(mt.r)) {
            s += cv::Point2d(cos(a), sin(a))*mt.l;
            path.poses.push_back(msgFromCv(s, a, path.poses.back().header));
            path.poses.back().header.stamp = rclcpp::Time(path.poses.back().header.stamp) + rclcpp::Duration::from_seconds(dur);
        }
        else {
            cv::Point2d mid = s + cv::Point2d(-sin(a), cos(a))*mt.r;
            double da = mt.l/mt.r;
            int cnt = std::abs(da)/(2*M_PI)*360/5;
            for (int i = 1; i < cnt; i++)
                path.poses.push_back(msgFromCv(mid - cv::Point2d(-sin(a+da/cnt*i), cos(a+da/cnt*i))*mt.r, normalize_theta(a+da/cnt*i), ps.header));
            a = normalize_theta(a + da);
            s = mid - cv::Point2d(-sin(a), cos(a))*mt.r;
            path.poses.push_back(msgFromCv(s, a, path.poses.back().header));
            path.poses.back().header.stamp = rclcpp::Time(path.poses.back().header.stamp) + rclcpp::Duration::from_seconds(dur);
        }
    }

    local_plan_pub_->publish(path);
}

void CscPursuitController::controlTimerCallback(const geometry_msgs::msg::PoseStamped &pose) {
    controlTimerStamp = pose.header.stamp;
    if ((rclcpp::Time(pose.header.stamp) - trajTime).seconds() > maxNoDataTime) {
        currentCmdVel = geometry_msgs::msg::TwistStamped();
        return;
    }

    geometry_msgs::msg::PoseStamped poseGlob;
    transformPose(traj.header.frame_id, pose, poseGlob);

    double minDist = std::numeric_limits<double>::infinity();
    double minDistS = 1;
    int minDistI = 0;

    tf2::Quaternion qRobot;
    tf2::convert(poseGlob.pose.orientation, qRobot);
    for (int i = beingVisitedI/*0*/; i <= std::min((int)traj.poses.size() - 2, beingVisitedI + maxTrajPosesSkip + 1); i++) {
        double s;
        double dist = segmentPointDistance(traj.poses[i].pose.position, traj.poses[i+1].pose.position, poseGlob.pose.position, &s);
        tf2::Quaternion qSeg;
        tf2::convert(traj.poses[i].pose.orientation, qSeg);
        double da = angularDistMultiplier*std::abs(qRobot.angleShortestPath(qSeg));
        dist = dist*dist + da*da;
        if (dist < minDist) {
            minDist = dist;
            minDistS = s;
            minDistI = i;
        }
    }

    // auto &last = traj.poses[traj.poses.size()-1].pose.position;
    // auto &prelast = traj.poses[traj.poses.size()-2].pose.position;
    // if (minDistI > traj.poses.size()-2
    //     || minDistI == traj.poses.size()-2
    //         && sqrt(sqr(last.x - prelast.x) + sqr(last.y - prelast.y))*(1-minDistS) < minTargetDistToFinish) {
    //     currentCmdVel = geometry_msgs::msg::Twist();
    //     return;
    // }

    beingVisitedI = minDistI;

    double distToControl = -minDistS;
    int controlI = std::min((int)traj.poses.size() - 1, beingVisitedI);
    while (controlI < (int)traj.poses.size()-1 && distToControl < minCarrotDist) {
      distToControl += sqrt(sqr(traj.poses[controlI+1].pose.position.x - traj.poses[controlI].pose.position.x)
                          + sqr(traj.poses[controlI+1].pose.position.y - traj.poses[controlI].pose.position.y));
      controlI++;
    }
    // RCLCPP_INFO(logger_, "bvi: %d (%lf, %lf), ci: %d (%lf, %lf)",
    //             beingVisitedI, traj.poses[beingVisitedI].pose.position.x, traj.poses[beingVisitedI].pose.position.y,
    //             controlI, traj.poses[controlI].pose.position.x, traj.poses[controlI].pose.position.y);

    auto &t = traj.poses[controlI].pose;
    tf2::Transform pt;
    tf2::convert(t, pt);

    tf2::Transform ps;
    tf2::convert(poseGlob.pose, ps);

    geometry_msgs::msg::TransformStamped poseGlobInv;
    tf2::convert(ps.inverse(), poseGlobInv.transform);
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(global_plan_pose, transformed_pose, poseGlobInv);
      return transformed_pose;
    };
    // Transform the near part of the global plan into the robot's frame of reference.
    std::vector<geometry_msgs::msg::PoseStamped> viaPoses;
    std::transform(
      traj.poses.begin() + beingVisitedI, traj.poses.begin() + (controlI + 1),
      std::back_inserter(viaPoses),
      transformGlobalPoseToLocal);

    int lastDir = rclcpp::Time(pose.header.stamp) - rclcpp::Time(currentCmdVel.header.stamp) > rclcpp::Duration::from_seconds(0.2)
                    ? 0
                    : sign(currentCmdVel.twist.linear.x);
    int dir = lastDir;
    int desiredDirChanges = 0;
    for (int i = 1; i < viaPoses.size(); i++) {
      auto &pose = viaPoses[i].pose;
      tf2::Vector3 mt(pose.position.x - viaPoses[i-1].pose.position.x, pose.position.y - viaPoses[i-1].pose.position.y, 0);
      tf2::Quaternion q;
      tf2::convert(pose.orientation, q);
      double angle = q.getAngle()*sign(q.getZ());
      // RCLCPP_INFO(logger_, "angle: %lf, pos: %lf %lf\n", angle, pose.position.x, pose.position.y);
      tf2::Vector3 ori(cos(angle), sin(angle), 0);
      mt.normalize(); //good only for logging
      int newDir = ori.dot(mt) >= 0 ? 1 : -1;
      if (dir != 0 && newDir != dir)
        desiredDirChanges++;
      dir = newDir;
      // RCLCPP_INFO(logger_, "cmt: %lf %lf (%lf %lf -> %lf %lf), ori: %lf %lf (%lf, %lf %lf), dir: %d, change: %d",
      //             mt.x(), mt.y(), viaPoses[i-1].pose.position.x, viaPoses[i-1].pose.position.y, pose.position.x, pose.position.y,
      //             ori.x(), ori.y(), angle/M_PI*180, q.getZ(), q.getW(), newDir, desiredDirChanges);
    }
    tf2::Transform psLoc;
    tf2::convert(pose.pose, psLoc);

    std::vector<Movement> mvmts;
    auto seq = movementsBetweenPoses(ps, pt,
                  lastDir,
                  desiredDirChanges,
                  viaPoses, &psLoc);
    mvmts.insert(mvmts.end(), seq.movements.begin(), seq.movements.end());
    planMovements(mvmts, controlI, sign(mvmts.back().l));
    visualizeMovements(mvmts, pose);
    control_start_pub_->publish(traj.poses[beingVisitedI]);
}

MovementsSequence CscPursuitController::movementsBetweenPoses(
          const tf2::Transform &ps, const tf2::Transform &pt, int lastDir, int desiredDirChangesCnt,
          const std::vector<geometry_msgs::msg::PoseStamped> &viaPoses, const tf2::Transform *psLoc) {
    cv::Point2d s;
    s.x = ps.getOrigin().x();
    s.y = ps.getOrigin().y();
    cv::Point2d vs;
    vs.x = ps.getBasis().getColumn(0).x();
    vs.y = ps.getBasis().getColumn(0).y();
    double vsmag = sqrt(vs.x*vs.x + vs.y*vs.y);
    vs.x /= vsmag;
    vs.y /= vsmag;
    cv::Point2d ns(-vs.y, vs.x);

    cv::Point2d t;
    t.x = pt.getOrigin().x();
    t.y = pt.getOrigin().y();
    cv::Point2d vt;
    vt.x = pt.getBasis().getColumn(0).x();
    vt.y = pt.getBasis().getColumn(0).y();
    double vtmag = sqrt(vt.x*vt.x + vt.y*vt.y);
    vt.x /= vtmag;
    vt.y /= vtmag;
    cv::Point2d nt(-vt.y, vt.x);

    std::vector<MovementsSequence> mvmts;
    MovementsSequence::fromPosesWithCircles(s, ns, t, nt, rrad, false, false, mvmts);
    MovementsSequence::fromPosesWithCircles(s, -ns, t, -nt, rrad, false, true, mvmts);
    MovementsSequence::fromPosesWithCircles(s, ns, t, -nt, rrad, true, false, mvmts);
    MovementsSequence::fromPosesWithCircles(s, -ns, t, nt, rrad, true, true, mvmts);

    double bestRes = std::numeric_limits<double>::infinity();
    int bestI = -1;
    std_msgs::msg::Float64MultiArray bestResiduals;
    bestResiduals.data.resize(5, 0);

    nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    std::vector<tf2::Vector3> footprint;
    if (!viaPoses.empty())
      for (auto &p : costmap_ros_->getRobotFootprint())
        footprint.emplace_back(tf2::Vector3(p.x, p.y, p.z));
    
    for (int i = 0; i < mvmts.size(); i++) {
        auto &mt = mvmts[i];
        double len = mt.length();
        std_msgs::msg::Float64MultiArray residuals;
        residuals.data.resize(6, 0);
        residuals.data[0] = len;
        if (!viaPoses.empty()) {
          double trajLen;
          residuals.data[1] = controlTrajErrPosPenalty*mt.trajectoryResidualsPositional(viaPoses, trajLen);
          residuals.data[0] = controlTrajErrLenPenalty*std::abs(len - trajLen);
          residuals.data[2] = controlTrajErrAnglePenalty*mt.trajectoryResidualsAngular(viaPoses, trajLen);
          if (psLoc != NULL)
            residuals.data[3] = controlTrajErrCostPenalty*mt.avgCost(costCheckStep, costCheckMetersPerRadian, footprint, *psLoc, *costmap);
        }
        residuals.data[4] = rradsPerDirChange*rrad*std::abs(mt.directionChangesCount(lastDir) - desiredDirChangesCnt);
        residuals.data[5] = rradsPerRRadChange*std::max(0.0, rrad - mt.minRotRadius());
        double res = 0;
        for (auto &r : residuals.data)
          res += r;
        if (res < bestRes) {
            bestRes = res;
            bestI = i;
            bestResiduals = residuals;
        }
    }
    if (bestI == -1)
      throw std::runtime_error("CscPursuitController movement not feasible!");
#ifdef CSC_PURSUIT_DEBUG
    if (!viaPoses.empty()) {
      // RCLCPP_INFO(logger_, "bestI: %d / %d, dcc: %d/%d", bestI, (int)mvmts.size(), mvmts[bestI].directionChangesCount(lastDir), desiredDirChangesCnt);
      residuals_pub_->publish(bestResiduals);
      double trajLen;
      // mvmts[bestI].trajectoryResidualsPositional(viaPoses, trajLen, &logger_);
      if (psLoc != NULL)
        mvmts[bestI].avgCost(costCheckStep, costCheckMetersPerRadian, footprint, *psLoc, *costmap, true, &logger_);
    }
#endif
    return mvmts[bestI];
}

void CscPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  traj = path;
  beingVisitedI = 0;
  trajTime = rclcpp::Time(path.header.stamp);
  trajChanged = true;
}

void CscPursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    maxVelLin = maxVelLinBase;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      maxVelLin = maxVelLinBase * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      maxVelLin = speed_limit;
    }
  }
}

double CscPursuitController::poseDistance(const geometry_msgs::msg::PoseStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2) {
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angularDistMultiplier*std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

nav_msgs::msg::Path CscPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (traj.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(traj.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist = max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    traj.poses.begin(), traj.poses.end(),
    [&robot_pose, this](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(robot_pose, ps);
    });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(traj.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = traj.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  traj.poses.erase(begin(traj.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool CscPursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace rtv_nav2_csc_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  rtv_nav2_csc_pursuit_controller::CscPursuitController,
  nav2_core::Controller)
