#include "planner/local_planner/teb.h"

#include <cmath>
#include <ctime>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "planner/local_planner/teb_types/misc.h"
#include "planner/local_planner/teb_types/pose_se2.h"

namespace local_planner {

using TebPose = teb_local_planner::PoseSE2;

TebLocalPlanner::TebLocalPlanner(std::shared_ptr<map::StaticMap> map,
                                 const rclcpp::Logger logger)
    : LocalPlannerBase(map, logger) {
  cfg_.trajectory.dt_ref = 0.3;
  cfg_.trajectory.min_samples = 10;
  cfg_.trajectory.max_samples = 500;
  cfg_.trajectory.estimate_orient = true;
  cfg_.robot.max_vel_x = 0.4;
  cfg_.robot.max_vel_x_backwards = 0.2;
  cfg_.robot.max_vel_theta = 1.0;
  cfg_.robot.acc_lim_x = 0.5;
  cfg_.robot.acc_lim_theta = 1.0;
  cfg_.optim.max_iterations = 50;
  cfg_.optim.penalty_epsilon = 0.1;
  cfg_.obstacles.min_obstacle_dist = 0.3;

  robot_model_ =
      std::make_shared<teb_local_planner::CircularRobotFootprintModel>();
  robot_model_->setRobotRadius(0.3);
}

void TebLocalPlanner::init() {
  initialized_ = false;
  prev_teb_poses_.clear();
  prev_teb_timediffs_.clear();
  teb_poses_.clear();
  teb_timediffs_.clear();
  teb_obstacles_.clear();
}

geometry_msgs::msg::Twist TebLocalPlanner::getControlCmd() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;

  if (smoothed_local_.poses.size() < 2) {
    RCLCPP_WARN(logger_, "TEB: No smoothed local path available");
    return cmd;
  }

  std::vector<Pose2D> ref_path;
  for (const auto& pose_stamped : smoothed_local_.poses) {
    Pose2D p;
    p.x = pose_stamped.pose.position.x;
    p.y = pose_stamped.pose.position.y;
    double qx = pose_stamped.pose.orientation.x;
    double qy = pose_stamped.pose.orientation.y;
    double qz = pose_stamped.pose.orientation.z;
    double qw = pose_stamped.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    p.theta = std::atan2(siny_cosp, cosy_cosp);
    ref_path.push_back(p);
  }

  Pose2D current_pose;
  current_pose.x = current_pose_.pose.position.x;
  current_pose.y = current_pose_.pose.position.y;
  double qx = current_pose_.pose.orientation.x;
  double qy = current_pose_.pose.orientation.y;
  double qz = current_pose_.pose.orientation.z;
  double qw = current_pose_.pose.orientation.w;
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  current_pose.theta = std::atan2(siny_cosp, cosy_cosp);

  Pose2D goal_pose = ref_path.back();

  convertMapObstacles();

  bool need_reinit = false;
  if (!isInitialized() || prev_teb_poses_.empty()) {
    need_reinit = true;
  } else {
    double pos_diff = distance(current_pose, prev_teb_poses_.front());
    double angle_diff = std::abs(teb_local_planner::normalizeAngle(
        current_pose.theta - prev_teb_poses_.front().theta));
    double goal_dist = distance(goal_pose, last_goal_pose_);
    if (pos_diff > 1.0 || angle_diff > 0.5 || goal_dist > 1.0) {
      need_reinit = true;
    }
  }

  if (need_reinit) {
    initTrajectory(ref_path);
    RCLCPP_DEBUG(logger_, "TEB: Initialized trajectory with %zu poses",
                 teb_poses_.size());
  } else {
    hotStart(ref_path, current_pose, goal_pose);
    RCLCPP_DEBUG(logger_, "TEB: Hot started with %zu poses", teb_poses_.size());
  }

  autoResize();

  buildGraph();

  if (!optimize()) {
    RCLCPP_WARN(logger_, "TEB: Optimization failed, using last valid command");
    return last_cmd_;
  }

  double v = 0.0, omega = 0.0;
  if (!extractVelocity(v, omega)) {
    RCLCPP_WARN(logger_, "TEB: Failed to extract velocity");
    return last_cmd_;
  }

  cmd.linear.x = v;
  cmd.angular.z = omega;
  last_cmd_ = cmd;

  prev_teb_poses_ = teb_poses_;
  prev_teb_timediffs_ = teb_timediffs_;
  initialized_ = true;
  last_goal_pose_ = goal_pose;

  RCLCPP_DEBUG(logger_, "TEB cmd: v=%.2f, omega=%.2f", v, omega);
  return cmd;
}

std::vector<Pose2D> TebLocalPlanner::getOptimizedTrajectory() const {
  return teb_poses_;
}

void TebLocalPlanner::initTrajectory(const std::vector<Pose2D>& ref_points) {
  int n = static_cast<int>(ref_points.size());
  if (n < 2) return;

  double total_length = 0.0;
  for (int i = 1; i < n; ++i) {
    total_length += distance(ref_points[i], ref_points[i - 1]);
  }

  double avg_vel = cfg_.robot.max_vel_x * 0.5;
  double total_time = (avg_vel > 1e-6) ? total_length / avg_vel : 10.0;
  int pose_count = std::max(
      cfg_.trajectory.min_samples,
      std::min(cfg_.trajectory.max_samples,
               static_cast<int>(total_time / cfg_.trajectory.dt_ref) + 1));

  teb_poses_.resize(pose_count);
  teb_timediffs_.assign(pose_count - 1, cfg_.trajectory.dt_ref);

  for (int i = 0; i < pose_count; ++i) {
    double t =
        (pose_count > 1) ? static_cast<double>(i) / (pose_count - 1) : 0.0;
    int idx = static_cast<int>(t * (n - 1));
    idx = std::min(idx, n - 2);
    double local_t = t * (n - 1) - idx;

    Pose2D p;
    p.x = ref_points[idx].x +
          local_t * (ref_points[idx + 1].x - ref_points[idx].x);
    p.y = ref_points[idx].y +
          local_t * (ref_points[idx + 1].y - ref_points[idx].y);

    if (cfg_.trajectory.estimate_orient && idx < n - 1) {
      double dx = ref_points[idx + 1].x - ref_points[idx].x;
      double dy = ref_points[idx + 1].y - ref_points[idx].y;
      p.theta = std::atan2(dy, dx);
    } else {
      p.theta = ref_points[idx].theta;
    }
    teb_poses_[i] = p;
  }
}

void TebLocalPlanner::hotStart(const std::vector<Pose2D>& ref_points,
                               const Pose2D& current_pose,
                               const Pose2D& goal_pose) {
  pruneTrajectory(current_pose);

  if (teb_poses_.empty()) {
    initTrajectory(ref_points);
    return;
  }

  teb_poses_.front() = current_pose;

  teb_poses_.back() = goal_pose;

  while (teb_timediffs_.size() < teb_poses_.size() - 1) {
    teb_timediffs_.push_back(cfg_.trajectory.dt_ref);
  }
  while (teb_timediffs_.size() > teb_poses_.size() - 1) {
    teb_timediffs_.pop_back();
  }

  if (teb_poses_.size() < static_cast<size_t>(cfg_.trajectory.min_samples)) {
    int extra = cfg_.trajectory.min_samples - teb_poses_.size();
    for (int i = 0; i < extra; ++i) {
      Pose2D mid;
      mid.x = (teb_poses_.front().x + teb_poses_.back().x) * 0.5;
      mid.y = (teb_poses_.front().y + teb_poses_.back().y) * 0.5;
      mid.theta = (teb_poses_.front().theta + teb_poses_.back().theta) * 0.5;
      if (teb_poses_.size() >= 2) {
        teb_poses_.insert(teb_poses_.begin() + 1, mid);
      } else {
        teb_poses_.insert(teb_poses_.begin() + 1, mid);
      }
      teb_timediffs_.insert(teb_timediffs_.begin(), cfg_.trajectory.dt_ref);
    }
  }
}

void TebLocalPlanner::autoResize() {
  if (teb_poses_.size() < 2) return;

  double dt_min = cfg_.trajectory.dt_ref * 0.3;
  double dt_max = cfg_.trajectory.dt_ref * 3.0;

  for (int loop = 0; loop < 5; ++loop) {
    bool changed = false;

    for (size_t i = 0; i < teb_timediffs_.size();) {
      double dt = teb_timediffs_[i];
      if (dt > dt_max) {
        if (teb_poses_.size() >=
            static_cast<size_t>(cfg_.trajectory.max_samples))
          break;
        Pose2D mid;
        mid.x = (teb_poses_[i].x + teb_poses_[i + 1].x) * 0.5;
        mid.y = (teb_poses_[i].y + teb_poses_[i + 1].y) * 0.5;
        mid.theta = teb_local_planner::normalizeAngle(
            (teb_poses_[i].theta + teb_poses_[i + 1].theta) * 0.5);
        teb_poses_.insert(teb_poses_.begin() + i + 1, mid);
        teb_timediffs_[i] = dt * 0.5;
        teb_timediffs_.insert(teb_timediffs_.begin() + i + 1, dt * 0.5);
        changed = true;
        i += 2;
      } else if (dt < dt_min &&
                 teb_poses_.size() >
                     static_cast<size_t>(cfg_.trajectory.min_samples)) {
        teb_poses_.erase(teb_poses_.begin() + i + 1);
        double merged_dt = dt;
        if (i + 1 < teb_timediffs_.size()) {
          merged_dt += teb_timediffs_[i + 1];
          teb_timediffs_.erase(teb_timediffs_.begin() + i + 1);
        }
        teb_timediffs_[i] = merged_dt;
        changed = true;
      } else {
        ++i;
      }
    }

    if (!changed) break;
  }
}

void TebLocalPlanner::buildGraph() {
  clearGraph();

  int n = static_cast<int>(teb_poses_.size());
  if (n < 2) return;

  pose_vec_.reserve(n);
  for (int i = 0; i < n; ++i) {
    auto* v = new teb_local_planner::VertexPose(
        teb_poses_[i].x, teb_poses_[i].y, teb_poses_[i].theta);
    if (i == 0) v->setFixed(true);
    optimizer_->addVertex(v);
    pose_vec_.push_back(v);
  }

  timediff_vec_.reserve(n - 1);
  for (int i = 0; i < n - 1; ++i) {
    auto* v = new teb_local_planner::VertexTimeDiff(teb_timediffs_[i]);
    optimizer_->addVertex(v);
    timediff_vec_.push_back(v);
  }

  double sc_shortest = cfg_.weights.weight_shortest_path;
  double sc_time = cfg_.weights.weight_time_optimal;
  double sc_vel = cfg_.weights.weight_velocity;
  double sc_acc = cfg_.weights.weight_acceleration;
  double sc_kin_nh = cfg_.weights.weight_kinematics_nh;
  double sc_obs = cfg_.weights.weight_obstacle;

  for (int i = 0; i < n - 1; ++i) {
    auto* edge_shortest = new teb_local_planner::EdgeShortestPath();
    edge_shortest->setVertex(0, pose_vec_[i]);
    edge_shortest->setVertex(1, pose_vec_[i + 1]);
    edge_shortest->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                  sc_shortest);
    edge_shortest->setTebConfig(cfg_);
    optimizer_->addEdge(edge_shortest);
  }

  for (int i = 0; i < n - 1; ++i) {
    auto* edge_time = new teb_local_planner::EdgeTimeOptimal();
    edge_time->setVertex(0, timediff_vec_[i]);
    edge_time->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                              sc_time);
    edge_time->setTebConfig(cfg_);
    optimizer_->addEdge(edge_time);
  }

  for (int i = 0; i < n - 1; ++i) {
    auto* edge_vel = new teb_local_planner::EdgeVelocity();
    edge_vel->setVertex(0, pose_vec_[i]);
    edge_vel->setVertex(1, pose_vec_[i + 1]);
    edge_vel->setVertex(2, timediff_vec_[i]);
    edge_vel->setInformation(Eigen::Matrix<double, 2, 2>::Identity() * sc_vel);
    edge_vel->setTebConfig(cfg_);
    optimizer_->addEdge(edge_vel);
  }

  for (int i = 0; i < n - 2; ++i) {
    auto* edge_acc = new teb_local_planner::EdgeAcceleration();
    edge_acc->setVertex(0, pose_vec_[i]);
    edge_acc->setVertex(1, pose_vec_[i + 1]);
    edge_acc->setVertex(2, pose_vec_[i + 2]);
    edge_acc->setVertex(3, timediff_vec_[i]);
    edge_acc->setVertex(4, timediff_vec_[i + 1]);
    edge_acc->setInformation(Eigen::Matrix<double, 2, 2>::Identity() * sc_acc);
    edge_acc->setTebConfig(cfg_);
    optimizer_->addEdge(edge_acc);
  }

  for (int i = 0; i < n - 1; ++i) {
    auto* edge_kin = new teb_local_planner::EdgeKinematicsDiffDrive();
    edge_kin->setVertex(0, pose_vec_[i]);
    edge_kin->setVertex(1, pose_vec_[i + 1]);
    edge_kin->setInformation(Eigen::Matrix<double, 2, 2>::Identity() *
                             sc_kin_nh);
    edge_kin->setTebConfig(cfg_);
    optimizer_->addEdge(edge_kin);
  }

  for (int i = 0; i < n; ++i) {
    for (const auto& obs : teb_obstacles_) {
      auto* edge_obs = new teb_local_planner::EdgeObstacle();
      edge_obs->setVertex(0, pose_vec_[i]);
      edge_obs->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                               sc_obs);
      edge_obs->setParameters(cfg_, robot_model_.get(), obs.get());
      optimizer_->addEdge(edge_obs);
    }
  }
}

bool TebLocalPlanner::optimize() {
  if (!optimizer_) return false;

  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(cfg_.optim.max_iterations);
  RCLCPP_DEBUG(logger_, "TEB optimization finished, iterations=%d", iter);

  for (size_t i = 0; i < pose_vec_.size(); ++i) {
    teb_poses_[i].x = pose_vec_[i]->x();
    teb_poses_[i].y = pose_vec_[i]->y();
    teb_poses_[i].theta = pose_vec_[i]->theta();
  }
  for (size_t i = 0; i < timediff_vec_.size(); ++i) {
    teb_timediffs_[i] = timediff_vec_[i]->dt();
  }

  return true;
}

bool TebLocalPlanner::extractVelocity(double& v, double& omega) {
  if (teb_poses_.size() < 2) return false;

  const auto& p0 = teb_poses_[0];
  const auto& p1 = teb_poses_[1];
  double dt = teb_timediffs_[0];

  if (dt < 0.05) dt = 0.05;

  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  v = std::sqrt(dx * dx + dy * dy) / dt;
  omega = teb_local_planner::normalizeAngle(p1.theta - p0.theta) / dt;

  if (v < 0.01) v = 0.0;
  if (std::abs(omega) < 0.01) omega = 0.0;

  v = std::min(v, cfg_.robot.max_vel_x);
  omega = std::max(-cfg_.robot.max_vel_theta,
                   std::min(omega, cfg_.robot.max_vel_theta));

  return true;
}

void TebLocalPlanner::clearGraph() {
  if (optimizer_) {
    optimizer_->clear();
    optimizer_.reset();
  }

  auto linear_solver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  auto block_solver =
      std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
  auto algorithm = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
      std::move(block_solver));

  optimizer_ = std::make_unique<g2o::SparseOptimizer>();
  optimizer_->setAlgorithm(algorithm.release());
  optimizer_->setVerbose(false);

  pose_vec_.clear();
  timediff_vec_.clear();
}

void TebLocalPlanner::pruneTrajectory(const Pose2D& current_pose) {
  if (teb_poses_.empty()) return;

  size_t prune_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < teb_poses_.size(); ++i) {
    double d = distance(teb_poses_[i], current_pose);
    if (d < min_dist) {
      min_dist = d;
      prune_idx = i;
    }
  }

  if (prune_idx > 0) {
    teb_poses_.erase(teb_poses_.begin(), teb_poses_.begin() + prune_idx);
    if (teb_timediffs_.size() > prune_idx) {
      teb_timediffs_.erase(teb_timediffs_.begin(),
                           teb_timediffs_.begin() + prune_idx);
    }
  }
}

double TebLocalPlanner::distance(const Pose2D& a, const Pose2D& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

void TebLocalPlanner::convertMapObstacles() {
  teb_obstacles_.clear();
  if (!map_) return;

  auto poly_obstacles = map_->obstacles();
  for (const auto& poly : poly_obstacles) {
    if (poly.size() >= 3) {
      teb_obstacles_.push_back(
          std::make_shared<teb_local_planner::PolygonObstacle>(poly));
    } else if (poly.size() == 1) {
      teb_obstacles_.push_back(
          std::make_shared<teb_local_planner::PointObstacle>(poly[0].x,
                                                             poly[0].y));
    }
  }
}

}  // namespace local_planner
