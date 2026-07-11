#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <vector>

#include "map/static_map.h"
#include "planner/local_planner/local_planner_base.h"

namespace local_planner {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner(std::shared_ptr<map::StaticMap> map,
                  const rclcpp::Logger logger)
      : LocalPlannerBase(map, logger) {}

  void init() override;

  geometry_msgs::msg::Twist getControlCmd() override;

  void setTrajectoryPoints(const std::vector<Pose2D>& points) {
    trajectory_points_ = points;
  }

  std::vector<Pose2D> getOptimizedTrajectory() const {
    return optimized_trajectory_;
  }

 private:
  struct OptimizerConfig {
    int n_points = 15;
    double dt_ref = 0.5;
    double wheelbase = 0.5;
    double max_linear_speed = 0.4;
    double max_angular_speed = 1.0;
    double safety_radius = 0.3;

    double w_obstacle = 100.0;
    double w_smoothness = 50.0;
    double w_velocity = 30.0;
    double w_time = 10.0;
    double w_kinematic = 20.0;

    int max_iterations = 50;
  };

  void initializePath(const std::vector<Pose2D>& ref_points);

  void buildQPProblem();

  bool solveQP();

  void extractOptimizedTrajectory();

  bool computeVelocityCommand(const Pose2D& p1, const Pose2D& p2, double& v,
                              double& omega);

  OptimizerConfig config_;

  std::vector<Pose2D> trajectory_points_;
  std::vector<Pose2D> initial_trajectory_;
  std::vector<Pose2D> optimized_trajectory_;

  int n_vars_;
  int n_constraints_;

  Eigen::VectorXd solution_;

  OsqpEigen::Solver solver_;
};

namespace {

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

}  // namespace

}  // namespace local_planner
