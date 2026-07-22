#include "planner/local_planner/teb.h"

#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace local_planner {

void TebLocalPlanner::init() {
  RCLCPP_INFO(logger_,
              "TebLocalPlanner initialized with %d points, dt_ref=%.2f",
              config_.n_points, config_.dt_ref);
}

void TebLocalPlanner::initializePath(const std::vector<Pose2D>& ref_points) {
  if (ref_points.size() < 2) {
    RCLCPP_WARN(logger_, "Reference points too few");
    return;
  }

  int n = ref_points.size();
  double total_length = 0.0;
  for (int i = 1; i < n; ++i) {
    double dx = ref_points[i].x - ref_points[i - 1].x;
    double dy = ref_points[i].y - ref_points[i - 1].y;
    total_length += std::sqrt(dx * dx + dy * dy);
  }

  config_.n_points = std::max(
      5, std::min(30, static_cast<int>(total_length / config_.dt_ref)));

  initial_trajectory_.resize(config_.n_points);

  for (int i = 0; i < config_.n_points; ++i) {
    double t = static_cast<double>(i) / (config_.n_points - 1);

    int idx = static_cast<int>(t * (n - 1));
    idx = std::min(idx, n - 2);

    double local_t = t * (n - 1) - idx;

    double x = ref_points[idx].x +
               local_t * (ref_points[idx + 1].x - ref_points[idx].x);
    double y = ref_points[idx].y +
               local_t * (ref_points[idx + 1].y - ref_points[idx].y);

    double dx = ref_points[idx + 1].x - ref_points[idx].x;
    double dy = ref_points[idx + 1].y - ref_points[idx].y;
    double theta = std::atan2(dy, dx);

    initial_trajectory_[i] = Pose2D{x, y, theta};
  }
}

void TebLocalPlanner::buildQPProblem() {
  int n = config_.n_points;
  int dim = 3;

  n_vars_ = n * dim + (n - 1);

  n_constraints_ = 0;
  n_constraints_ += (n - 1) * 3;
  n_constraints_ += (n - 1) * 2;
  n_constraints_ += (n - 1) * 1;
  if (!map_->obstacles().empty()) {
    n_constraints_ +=
        n * std::min(static_cast<int>(map_->obstacles().size()), 10);
  }

  Eigen::SparseMatrix<double> hessian(n_vars_, n_vars_);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_vars_);
  Eigen::SparseMatrix<double> constraint_matrix(n_constraints_, n_vars_);
  Eigen::VectorXd lower_bound(n_constraints_);
  Eigen::VectorXd upper_bound(n_constraints_);

  for (int i = 0; i < n - 1; ++i) {
    int base_idx = i * dim;
    hessian.insert(base_idx, base_idx) = config_.w_smoothness;
    hessian.insert(base_idx + 1, base_idx + 1) = config_.w_smoothness;
    hessian.insert(base_idx + 2, base_idx + 2) = config_.w_smoothness * 0.1;
  }

  int dt_base = n * dim;
  for (int i = 0; i < n - 1; ++i) {
    int dt_idx = dt_base + i;
    hessian.insert(dt_idx, dt_idx) = config_.w_time;
  }

  hessian = hessian + Eigen::SparseMatrix<double>(hessian.transpose()) -
            Eigen::SparseMatrix<double>(hessian.diagonal().asDiagonal());
  hessian.makeCompressed();

  int cons_idx = 0;

  for (int i = 0; i < n - 1; ++i) {
    int base_i = i * dim;
    int base_j = (i + 1) * dim;

    constraint_matrix.insert(cons_idx, base_i) = 1.0;
    constraint_matrix.insert(cons_idx, base_j) = -1.0;
    lower_bound(cons_idx) = -1.0;
    upper_bound(cons_idx) = 1.0;
    cons_idx++;

    constraint_matrix.insert(cons_idx, base_i + 1) = 1.0;
    constraint_matrix.insert(cons_idx, base_j + 1) = -1.0;
    lower_bound(cons_idx) = -1.0;
    upper_bound(cons_idx) = 1.0;
    cons_idx++;

    constraint_matrix.insert(cons_idx, base_i + 2) = 1.0;
    constraint_matrix.insert(cons_idx, base_j + 2) = -1.0;
    lower_bound(cons_idx) = -M_PI;
    upper_bound(cons_idx) = M_PI;
    cons_idx++;
  }

  for (int i = 0; i < n - 1; ++i) {
    int base_i = i * dim;
    int base_j = (i + 1) * dim;
    int dt_idx = dt_base + i;

    double dx = initial_trajectory_[i + 1].x - initial_trajectory_[i].x;
    double dy = initial_trajectory_[i + 1].y - initial_trajectory_[i].y;
    double dist = std::sqrt(dx * dx + dy * dy) + 1e-6;

    constraint_matrix.insert(cons_idx, base_j) = 1.0;
    constraint_matrix.insert(cons_idx, base_i) = -1.0;
    constraint_matrix.insert(cons_idx, dt_idx) = -config_.max_linear_speed;
    lower_bound(cons_idx) = -dist;
    upper_bound(cons_idx) = dist;
    cons_idx++;

    double dtheta = normalizeAngle(initial_trajectory_[i + 1].theta -
                                   initial_trajectory_[i].theta);
    constraint_matrix.insert(cons_idx, base_j + 2) = 1.0;
    constraint_matrix.insert(cons_idx, base_i + 2) = -1.0;
    constraint_matrix.insert(cons_idx, dt_idx) = -config_.max_angular_speed;
    lower_bound(cons_idx) = dtheta - 0.1;
    upper_bound(cons_idx) = dtheta + 0.1;
    cons_idx++;
  }

  for (int i = 0; i < n - 1; ++i) {
    int dt_idx = dt_base + i;
    constraint_matrix.insert(cons_idx, dt_idx) = 1.0;
    lower_bound(cons_idx) = 0.1;
    upper_bound(cons_idx) = 5.0;
    cons_idx++;
  }

  auto obstacles = map_->obstacles();
  if (!obstacles.empty()) {
    int num_obs = std::min(static_cast<int>(obstacles.size()), 10);
    for (int obs_idx = 0; obs_idx < num_obs; ++obs_idx) {
      const auto& obs = obstacles[obs_idx];
      if (obs.empty()) continue;

      double obs_x = 0.0, obs_y = 0.0;
      for (const auto& node : obs) {
        obs_x += node.x;
        obs_y += node.y;
      }
      obs_x /= obs.size();
      obs_y /= obs.size();

      for (int i = 0; i < n; ++i) {
        int base_i = i * 3;

        double px = initial_trajectory_[i].x;
        double py = initial_trajectory_[i].y;

        constraint_matrix.insert(cons_idx, base_i) = 2.0 * (px - obs_x);
        constraint_matrix.insert(cons_idx, base_i + 1) = 2.0 * (py - obs_y);
        lower_bound(cons_idx) = -Eigen::Infinity;
        upper_bound(cons_idx) = pow(config_.safety_radius + 0.2, 2) -
                                (px - obs_x) * (px - obs_x) -
                                (py - obs_y) * (py - obs_y);
        cons_idx++;
      }
    }
  }

  constraint_matrix.makeCompressed();

  solver_.settings()->setWarmStart(true);
  solver_.settings()->setMaxIteration(config_.max_iterations);
  solver_.settings()->setRelativeTolerance(1e-4);

  solver_.data()->setNumberOfVariables(n_vars_);
  solver_.data()->setNumberOfConstraints(n_constraints_);

  solver_.data()->setHessianMatrix(hessian);
  solver_.data()->setGradient(gradient);
  solver_.data()->setLinearConstraintsMatrix(constraint_matrix);
  solver_.data()->setLowerBound(lower_bound);
  solver_.data()->setUpperBound(upper_bound);
}

bool TebLocalPlanner::solveQP() {
  if (!solver_.initSolver()) {
    RCLCPP_ERROR(logger_, "Failed to initialize OSQP solver");
    return false;
  }

  auto status = solver_.solveProblem();
  if (status != OsqpEigen::ErrorExitFlag::NoError) {
    RCLCPP_ERROR(logger_, "Failed to solve QP, error flag: %d",
                 static_cast<int>(status));
    return false;
  }

  solution_ = solver_.getSolution();
  return true;
}

void TebLocalPlanner::extractOptimizedTrajectory() {
  optimized_trajectory_.clear();

  int n = config_.n_points;
  for (int i = 0; i < n; ++i) {
    int base_idx = i * 3;
    Pose2D p;
    p.x = solution_(base_idx);
    p.y = solution_(base_idx + 1);
    p.theta = solution_(base_idx + 2);
    optimized_trajectory_.push_back(p);
  }
}

bool TebLocalPlanner::computeVelocityCommand(const Pose2D& p1, const Pose2D& p2,
                                             double& v, double& omega) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dtheta = normalizeAngle(p2.theta - p1.theta);

  int dt_base = config_.n_points * 3;
  double dt = config_.dt_ref;
  if (solution_.size() > dt_base) {
    dt = solution_(dt_base);
  }
  if (dt < 1e-6) dt = 1e-6;

  v = std::sqrt(dx * dx + dy * dy) / dt;
  omega = dtheta / dt;

  v = std::max(0.0, std::min(v, config_.max_linear_speed));
  omega = std::max(-config_.max_angular_speed,
                   std::min(omega, config_.max_angular_speed));

  return true;
}

geometry_msgs::msg::Twist TebLocalPlanner::getControlCmd() {
  geometry_msgs::msg::Twist cmd;

  if (smoothed_local_.poses.empty()) {
    RCLCPP_WARN(logger_, "No smoothed local path available");
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  trajectory_points_.clear();
  for (const auto& pose : smoothed_local_.poses) {
    Pose2D p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.theta = 0.0;
    trajectory_points_.push_back(p);
  }

  if (trajectory_points_.size() < 2) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  initializePath(trajectory_points_);

  if (initial_trajectory_.empty()) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  solution_ = Eigen::VectorXd(n_vars_);

  for (int i = 0; i < config_.n_points; ++i) {
    int base_idx = i * 3;
    solution_(base_idx) = initial_trajectory_[i].x;
    solution_(base_idx + 1) = initial_trajectory_[i].y;
    solution_(base_idx + 2) = initial_trajectory_[i].theta;
    if (i < config_.n_points - 1) {
      solution_(config_.n_points * 3 + i) = config_.dt_ref;
    }
  }

  buildQPProblem();

  if (!solveQP()) {
    RCLCPP_WARN(logger_, "QP solving failed, using initial trajectory");
    optimized_trajectory_ = initial_trajectory_;
  } else {
    extractOptimizedTrajectory();
  }

  if (optimized_trajectory_.size() < 2) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  double v = 0.0, omega = 0.0;
  size_t lookAheadIdx =
      std::min(3, static_cast<int>(optimized_trajectory_.size() - 1));

  computeVelocityCommand(optimized_trajectory_[0],
                         optimized_trajectory_[lookAheadIdx], v, omega);

  cmd.linear.x = v;
  cmd.angular.z = omega;

  return cmd;
}

}  // namespace local_planner
