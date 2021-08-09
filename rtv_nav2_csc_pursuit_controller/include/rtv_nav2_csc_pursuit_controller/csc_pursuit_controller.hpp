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

#ifndef RTV_NAV2_CSC_PURSUIT_CONTROLLER__ARCWISE_FOLLOW_PLAN_CONTROLLER_HPP_
#define RTV_NAV2_CSC_PURSUIT_CONTROLLER__ARCWISE_FOLLOW_PLAN_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "opencv2/core.hpp"


namespace rtv_nav2_csc_pursuit_controller
{

struct Movement {
    double r;
    double l;

    Movement(double r, double l) : r(r), l(l) {}
};



struct MovementsSequence {

    std::vector<Movement> movements;

    static void fromPosesWithCircles(cv::Point2d s, cv::Point2d ns, cv::Point2d t, cv::Point2d nt,
                                     double r, bool inner, bool reverseDir, std::vector<MovementsSequence> &output);

    MovementsSequence() {}
    inline MovementsSequence(double das, double dasSign, double dat, double datSign,
                     double tsign, double r, double ds, double dsSign, double sdir) {
        if (das*dasSign < -0.001)
            das += 2*M_PI*dasSign;
        if (das*dasSign > 0.001)
            movements.emplace_back(r*sdir, r*das*sdir);
        if (ds > 0.001)
            movements.emplace_back(std::numeric_limits<double>::infinity(), ds*dsSign*sdir);
        if (dat*datSign < -0.001)
            dat += 2*M_PI*datSign;//
        if (dat*datSign > 0.001)
            movements.emplace_back(r*tsign*sdir, r*dat*tsign*sdir);
        // printf("%lf %lf %lf\n", das, ds, dat);
    }

    double length();

    int directionChangesCount(int lastDir = 0);

    double minRotRadius();

    double trajectoryResidualsPositional(const std::vector<geometry_msgs::msg::PoseStamped> &traj, double &trajLen, rclcpp::Logger *logger = NULL);
    double trajectoryResidualsAngular(const std::vector<geometry_msgs::msg::PoseStamped> &traj, double trajLen);
    double avgCost(double maxPosDiff, double distPerRot, const std::vector<tf2::Vector3> &footprint,
                   const tf2::Transform& pose, const nav2_costmap_2d::Costmap2D &costmap, bool visualize = false, rclcpp::Logger *logger = NULL);
};

/**
 * @class rtv_nav2_csc_pursuit_controller::CscPursuitController
 * @brief CSC pursuit controller plugin
 */
class CscPursuitController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for rtv_nav2_csc_pursuit_controller::CscPursuitController
   */
  CscPursuitController() = default;

  /**
   * @brief Destructor for rtv_nav2_csc_pursuit_controller::CscPursuitController
   */
  ~CscPursuitController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

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

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
   * @param pose pose to transform
   * @return Path in new frame
   */
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Transform a pose to another frame.
   * @param frame Frame ID to transform to
   * @param in_pose Pose input to transform
   * @param out_pose transformed output
   * @return bool if successful
   */
  bool transformPose(
    const std::string frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  /**
   * @brief Creates a PointStamped message for visualization
   * @param carrot_pose Input carrot point as a PoseStamped
   * @return CarrotMsg a carrot point marker, PointStamped
   */
  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Whether robot should rotate to rough path heading
   * @param carrot_pose current lookahead point
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @return Whether should rotate to path heading
   */
  bool shouldRotateToPath(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path);

  /**
   * @brief Whether robot should rotate to final goal orientation
   * @param carrot_pose current lookahead point
   * @return Whether should rotate to goal heading
   */
  bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped & carrot_pose);

  /**
   * @brief Create a smooth and kinematically smoothed rotation command
   * @param linear_vel linear velocity
   * @param angular_vel angular velocity
   * @param angle_to_path Angle of robot output relatie to carrot marker
   * @param curr_speed the current robot speed
   */
  void rotateToHeading(
    double & linear_vel, double & angular_vel,
    const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed);

  /**
   * @brief Whether collision is imminent
   * @param robot_pose Pose of robot
   * @param carrot_pose Pose of carrot
   * @param linear_vel linear velocity to forward project
   * @param angular_vel angular velocity to forward project
   * @return Whether collision is imminent
   */
  bool isCollisionImminent(
    const geometry_msgs::msg::PoseStamped &,
    const double &, const double &);

  /**
   * @brief Whether point is in collision
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Whether in collision
   */
  bool inCollision(const double & x, const double & y);

  /**
   * @brief Cost at a point
   * @param x Pose of pose x
   * @param y Pose of pose y
   * @return Cost of pose in costmap
   */
  double costAtPose(const double & x, const double & y);

  /**
   * @brief apply regulation constraints to the system
   * @param linear_vel robot command linear velocity input
   * @param dist_error error in the carrot distance and lookahead distance
   * @param lookahead_dist optimal lookahead distance
   * @param curvature curvature of path
   * @param speed Speed of robot
   * @param pose_cost cost at this pose
   */
  void applyConstraints(
    const double & dist_error, const double & lookahead_dist,
    const double & curvature, const geometry_msgs::msg::Twist & speed,
    const double & pose_cost, double & linear_vel);

  /**
   * @brief Get lookahead point
   * @param lookahead_dist Optimal lookahead distance
   * @param path Current global path
   * @return Lookahead point
   */
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

  void planMovements(std::vector<Movement> &mvmts, int starti, int lastDir);
  void joinShortMovements(std::vector<Movement> &mvmts);
  void repeatTimerCallback(const rclcpp::Time &now);
  void visualizeMovements(const std::vector<Movement> &mvmts, const geometry_msgs::msg::PoseStamped &ps);
  void controlTimerCallback(const geometry_msgs::msg::PoseStamped &pose);
  double poseDistance(const geometry_msgs::msg::PoseStamped &pose1, const geometry_msgs::msg::PoseStamped &pose2);
  MovementsSequence movementsBetweenPoses(const tf2::Transform &ps, const tf2::Transform &pt, int lastDir,
                                          int desiredDirChangesCnt,
                                          const std::vector<geometry_msgs::msg::PoseStamped> &viaPoses = std::vector<geometry_msgs::msg::PoseStamped>(),
                                          const tf2::Transform *psLoc = NULL);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("CscPursuitController")};

  std::string baseFootprintFrame;
  double controlInterval;
  double repeatInterval;
  double maxNoDataTime;

  double maxVelLin;
  double maxVelLinBase;
  double maxVelLinDecPerVelRot;
  double maxVelRot;
  double maxVelRotDecPerRRadDec;
  double minMaxVelRot;
  double rrad;
  double minTargetDistToFinish;
  double minCarrotDist;

  double rradsPerDirChange;
  double rradsPerRRadChange;
  double minMvmtDur;
  int maxTrajPosesSkip;
  double controlTrajErrPosPenalty;
  double controlTrajErrLenPenalty;
  double controlTrajErrAnglePenalty;
  double controlTrajErrCostPenalty;
  double costCheckStep;
  double costCheckMetersPerRadian;

  double angularDistMultiplier;

  rclcpp::Time controlTimerStamp;
  rclcpp::Time repeatTimerStamp;

  bool shouldControl;
  bool blindStart;

  tf2::Duration transform_tolerance_;

  rclcpp::Time trajTime;
  nav_msgs::msg::Path traj;
  bool trajChanged = false;
  int beingVisitedI;
  std::vector<Movement> movements;

  int currentMovement;
  rclcpp::Time currentMovementEnd;
  geometry_msgs::msg::TwistStamped currentCmdVel;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>> residuals_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> control_start_pub_;
};

}  // namespace rtv_nav2_csc_pursuit_controller

#endif  // RTV_NAV2_CSC_PURSUIT_CONTROLLER__ARCWISE_FOLLOW_PLAN_CONTROLLER_HPP_
