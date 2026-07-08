#include "planner/local_planner/dwa.h"

#include "common/util.h"

namespace local_planner {

void LocalPlannerDWA::init() {
  max_v_ = 1.0;
  min_v_ = 0.0;
  max_omega_ = 1.8;
  min_omega_ = -1.8;
  max_acc_v_ = 5.0;
  max_decl_v_ = 5.0;
  max_acc_omega_ = 5.0;
  max_decl_omega_ = 5.0;

  v_resolution_ = 0.1;
  omega_resolution_ = 0.1;

  dt_ = 0.1;
  total_time_ = 5.0;
  lookahead_dist_ = 1.0;

  safe_radius = 8.0;

  weight_heading_ = 2.0;
  weight_velocity_ = 0.5;
  weight_obstacle_ = 1.0;
  weight_dist_ = 1.0;

  sim_step_ = total_time_ / dt_;
}

Node3D LocalPlannerDWA::motionModel(const Node3D& start, const double v,
                                    const double omega) {
  // 差速驱动 运动学更新
  Node3D next;
  if (std::fabs(omega) < eplison) {
    next.setX(start.getX() + v * dt_ * std::cos(start.getT()));
    next.setY(start.getY() + v * dt_ * std::sin(start.getT()));
    next.setT(start.getT());
  } else {
    double r = v / omega;
    double theta_new = start.getT() + omega * dt_;
    next.setX(start.getX() +
              r * (std::sin(theta_new) - std::sin(start.getT())));
    next.setY(start.getY() -
              r * (std::cos(theta_new) - std::cos(start.getT())));
    if (theta_new > M_PI) {
      theta_new -= 2 * M_PI;
    } else if (theta_new < -M_PI) {
      theta_new += 2 * M_PI;
    }
    next.setT(theta_new);
  }
  return next;
}

std::vector<Node3D> LocalPlannerDWA::predictTrajectory(
    const double v, const double omega, const Node3D start_node) {
  std::vector<Node3D> traj;
  traj.emplace_back(start_node);
  const auto& obstacles = map_->obstacles();

  for (int i = 0; i < sim_step_; ++i) {
    Node3D next = motionModel(traj.back(), v, omega);
    traj.emplace_back(next);
  }
  return traj;
}

bool LocalPlannerDWA::isSafe(const std::vector<Node3D>& traj, const double v,
                             const double omega, double& min_d) {
  const std::vector<std::vector<Node3D>> obstacles = map_->obstacles();
  double min_dist = std::numeric_limits<double>::infinity();
  for (const Node3D& node : traj) {
    for (const auto& obs : obstacles) {
      double dist = pointToPolygonDistance(node, obs);
      min_dist = std::min(dist, min_dist);
    }
  }

  min_d = min_dist;
  if (min_dist < 1e-3) {
    return false;
  }

  double v_limit = std::sqrt(2 * min_dist * max_decl_v_);
  double omega_limit = std::sqrt(2 * min_dist * max_decl_omega_);

  if (std::fabs(v) > v_limit || std::fabs(omega) > omega_limit) {
    return false;
  }
  return true;
}

DWACost LocalPlannerDWA::computeCost(const std::vector<Node3D>& traj, double v,
                                     const Node3D& goal,
                                     const double min_dist) {
  // 1. heading cost
  const Node3D& end_pt = traj.back();
  double goal_theta =
      std::atan2(goal.getY() - end_pt.getY(), goal.getX() - end_pt.getX());
  double heading_err =
      std::fabs(std::atan2(std::sin(end_pt.getT() - goal_theta),
                           std::cos(end_pt.getT() - goal_theta)));
  double cost_heading = weight_heading_ * heading_err * (v / max_v_);

  // 2. velocity_cost
  double cost_vel = weight_velocity_ * (max_v_ - v);

  // 3. obstacle_cost
  double cost_obs = weight_obstacle_ * (1.0 / (min_dist + 1e6));

  // 4. dist cost
  double dx = goal.getX() - end_pt.getX();
  double dy = goal.getY() - end_pt.getY();
  double cost_dist = std::hypot(dx, dy) * weight_dist_ * (v < 0.1 ? 5.0 : 1.0);
  double total_cost = cost_heading + cost_vel + cost_obs + cost_dist;

  return DWACost{cost_heading, cost_vel, cost_obs, cost_dist, total_cost};
}

geometry_msgs::msg::PoseStamped LocalPlannerDWA::getNearGoal() {
  double arc_len = 0.0;
  const int n = smoothed_local_.poses.size();
  int idx = n - 1;
  for (int i = 1; i < n; ++i) {
    double dx = smoothed_local_.poses[i].pose.position.x -
                smoothed_local_.poses[i - 1].pose.position.x;
    double dy = smoothed_local_.poses[i].pose.position.y -
                smoothed_local_.poses[i - 1].pose.position.y;

    arc_len += std::hypot(dx, dy);
    if (arc_len >= lookahead_dist_) {
      idx = i;
      break;
    }
  }
  return smoothed_local_.poses[idx];
}

geometry_msgs::msg::Twist LocalPlannerDWA::getControlCmd() {
  sampled_trajectories_.clear();
  int best_traj_idx = -1;

  Node3D start_node = Node3D(
      current_pose_.pose.position.x, current_pose_.pose.position.y,
      common::yawFromQuaternion(current_pose_.pose.orientation), 0, 0, nullptr);
  geometry_msgs::msg::PoseStamped goal = getNearGoal();
  Node3D goal_node =
      Node3D(goal.pose.position.x, goal.pose.position.y,
             common::yawFromQuaternion(goal.pose.orientation), 0, 0, nullptr);

  double min_cost = std::numeric_limits<double>::infinity();

  double best_v = 0.0;
  double best_omega = 0.0;
  std::vector<Node3D> best_traj;

  // 1. 设置上下限窗口
  double min_v_win = min_v_;
  double max_v_win = max_v_;
  double min_omega_win = min_omega_;
  double max_omega_win = max_omega_;

  min_v_win =
      std::max(min_v_win, current_twist_.twist.linear.z - max_acc_v_ * dt_);
  max_v_win =
      std::min(max_v_win, current_twist_.twist.linear.z + max_acc_v_ * dt_);
  min_omega_win = std::max(
      min_omega_win, current_twist_.twist.angular.z - max_acc_omega_ * dt_);
  max_omega_win = std::min(
      max_omega_win, current_twist_.twist.angular.z + max_acc_omega_ * dt_);

  // 2. 遍历采样
  for (double v = min_v_win; v < max_v_win; v += v_resolution_) {
    for (double omega = min_omega_win; omega < max_omega_win;
         omega += omega_resolution_) {
      std::vector<Node3D> traj = predictTrajectory(v, omega, start_node);
      double min_dist;
      if (!isSafe(traj, v, omega, min_dist)) {
        continue;
      }

      DWACost cost = computeCost(traj, v, goal_node, min_dist);

      TrajectorySample sample;
      sample.nodes = traj;
      sample.v = v;
      sample.omega = omega;
      sample.cost = cost;
      sample.is_best = false;
      sampled_trajectories_.push_back(sample);

      if (cost.toal_cost < min_cost) {
        min_cost = cost.toal_cost;
        best_v = v;
        best_omega = omega;
        best_traj = traj;
        best_traj_idx = sampled_trajectories_.size() - 1;
      }
    }
  }
  if (best_traj_idx != -1) {
    sampled_trajectories_[best_traj_idx].is_best = true;
  }
  // 3. 生成结果
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = best_v;
  cmd.angular.z = best_omega;
  return cmd;
}

void LocalPlannerDWA::visualizeSampledTrajectories(
    const std::string& frame_id) {
  auto& vis = visualization::VisualizationManager::Instance();
  vis.ClearNamespace("dwa_trajectories");

  visualization::Color best_color(0.0f, 1.0f, 0.0f, 1.0f);
  visualization::Color other_color(0.0f, 0.0f, 0.0f, 1.0f);

  int id = 0;
  for (const auto& sample : sampled_trajectories_) {
    if (sample.nodes.size() < 2) {
      continue;
    }

    std::vector<geometry_msgs::msg::Point> points;

    for (const auto& node : sample.nodes) {
      geometry_msgs::msg::Point pt;
      pt.x = node.getX();
      pt.y = node.getY();
      pt.z = 0.0;
      points.push_back(pt);
    }

    const auto& color = sample.is_best ? best_color : other_color;
    vis.AddLineStrip("dwa_trajectories", id++, points, color, 0.02f, frame_id);
  }
}

}  // namespace local_planner