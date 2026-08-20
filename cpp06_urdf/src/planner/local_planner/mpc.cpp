#include "planner/local_planner/mpc.h"

#include <cmath>
#include <limits>

namespace local_planner {
void LocalPlannerMPC::init(const int nx, const int nu, const int N, double dt,
                           double v_ref) {
  nx_ = nx;
  nu_ = nu;
  N_ = N;
  dt_ = dt;
  v_ref_ = v_ref;
  n_var_ = (N + 1) * nx + N * nu;
  n_constr_ = (N + 1) * nx + n_var_;

  // 权重矩阵初始化
  Q_ = Eigen::MatrixXd::Zero(nx_, nx_);
  Q_(0, 0) = 500.0;  // x误差惩罚
  Q_(1, 1) = 500.0;  // y误差惩罚
  Q_(2, 2) = 50.0;   // theta 误差惩罚

  R_ = Eigen::MatrixXd::Zero(nu_, nu_);
  R_(0, 0) = 10.0;  // v增量惩罚
  R_(1, 1) = 5.0;   // omega 增量惩罚

  P_ = Q_;

  // 状态/控制boundary
  x_min_ = Eigen::VectorXd::Constant(nx_, -10.0);
  x_max_ = Eigen::VectorXd::Constant(nx_, 10.0);
  x_min_(2) = -M_PI;
  x_max_(2) = M_PI;
  u_min_ = Eigen::VectorXd::Constant(nu_, -1e3);
  u_max_ = Eigen::VectorXd::Constant(nu_, 1e3);
  u_min_(0) = -0.2;
  u_max_(0) = 0.6;

  u_min_(1) = -1.8;
  u_max_(1) = 1.8;

  solver_initialized_ = false;
}

std::vector<PathPoint> LocalPlannerMPC::extractPathPoints(
    const nav_msgs::msg::Path& path) {
  std::vector<PathPoint> pts;
  for (const auto& pose_stamped : path.poses) {
    PathPoint pt;
    pt.x = pose_stamped.pose.position.x;
    pt.y = pose_stamped.pose.position.y;
    pt.theta = common::yawFromQuaternion(pose_stamped.pose.orientation);
    pts.push_back(pt);
  }
  return pts;
}

// 2. 计算累计弧长
std::vector<double> LocalPlannerMPC::computeArcLength(
    const std::vector<PathPoint>& pts) {
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    double dx = pts[i].x - pts[i - 1].x;
    double dy = pts[i].y - pts[i - 1].y;
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  return s;
}

// 3. 按时间步长重采样
std::vector<PathPoint> LocalPlannerMPC::resamplePath(
    const std::vector<PathPoint>& pts, const std::vector<double>& s) {
  std::vector<PathPoint> ref_traj;
  assert(s.size() >= 3);
  double total_length = s.back();

  for (int k = 0; k <= N_; ++k) {
    double sk = k * v_ref_ * dt_;
    if (sk >= total_length) {
      sk = total_length;
    }

    size_t i = 0;
    const auto& it = std::upper_bound(s.begin(), s.end() - 1, sk);
    if (it != s.end()) {
      i = (it - s.begin()) == 0 ? 0 : (it - s.begin() - 1);
    } else {
      i = s.size() - 2;
    }

    double ratio = (sk - s[i]) / (s[i + 1] - s[i] + epsilon);
    PathPoint pt;
    pt.x = pts[i].x + ratio * (pts[i + 1].x - pts[i].x);
    pt.y = pts[i].y + ratio * (pts[i + 1].y - pts[i].y);
    double dtheta = pts[i + 1].theta - pts[i].theta;
    dtheta = common::normalizeAngleDiff(dtheta);
    pt.theta = pts[i].theta + ratio * dtheta;

    ref_traj.push_back(pt);
  }
  return ref_traj;
}

// 4. 计算参考线控制量
std::vector<Control> LocalPlannerMPC::computeReferenceControls(
    const std::vector<PathPoint>& ref_traj) {
  std::vector<Control> u_ref;
  for (size_t k = 1; k < ref_traj.size(); ++k) {
    Control u;
    double dx = ref_traj[k].x - ref_traj[k - 1].x;
    double dy = ref_traj[k].y - ref_traj[k - 1].y;
    double dtheta = ref_traj[k].theta - ref_traj[k - 1].theta;
    dtheta = common::normalizeAngleDiff(dtheta);

    u.v = std::hypot(dx, dy) / dt_;
    u.omega = dtheta / dt_;
    u_ref.push_back(u);
  }

  u_ref.push_back(u_ref.back());
  return u_ref;
}

std::vector<PathPoint> LocalPlannerMPC::rebuildReferenceFromEgo(
    const std::vector<PathPoint>& pts, double ego_x, double ego_y,
    double ego_yaw) {
  if (pts.size() < 2) {
    return pts;
  }

  // 1. 逐段投影，找自车到参考线最近的点
  size_t best_seg = 0;
  double best_t = 0.0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    double dx = pts[i + 1].x - pts[i].x;
    double dy = pts[i + 1].y - pts[i].y;
    double len2 = dx * dx + dy * dy;
    if (len2 < epsilon) {
      continue;
    }
    double t = ((ego_x - pts[i].x) * dx + (ego_y - pts[i].y) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    double dist =
        std::hypot(pts[i].x + t * dx - ego_x, pts[i].y + t * dy - ego_y);
    if (dist < best_dist) {
      best_dist = dist;
      best_seg = i;
      best_t = t;
    }
  }

  // 2. 拼接：自车位姿 → 投影点 → 原路径剩余部分
  PathPoint ego;
  ego.x = ego_x;
  ego.y = ego_y;
  ego.theta = ego_yaw;

  std::vector<PathPoint> rebuilt;
  rebuilt.push_back(ego);

  if (best_dist >= 1e-3) {
    PathPoint proj;
    proj.x = pts[best_seg].x + best_t * (pts[best_seg + 1].x - pts[best_seg].x);
    proj.y = pts[best_seg].y + best_t * (pts[best_seg + 1].y - pts[best_seg].y);
    double dtheta = common::normalizeAngleDiff(pts[best_seg + 1].theta -
                                               pts[best_seg].theta);
    proj.theta = pts[best_seg].theta + best_t * dtheta;
    rebuilt.push_back(proj);
  }

  size_t start = best_seg + 1;
  if (best_t > 1.0 - 1e-6 && start + 1 < pts.size() && rebuilt.size() > 1) {
    ++start;  // 投影点与 pts[best_seg + 1] 重合，避免重复
  }
  for (size_t i = start; i < pts.size(); ++i) {
    rebuilt.push_back(pts[i]);
  }

  // 3. 端点固定平滑：首点（自车）与尾点不动，内部点窗口均值
  const int iterations = 3;
  const int half_window = 2;
  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<PathPoint> smoothed = rebuilt;
    for (size_t i = 1; i + 1 < rebuilt.size(); ++i) {
      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_sin = 0.0;
      double sum_cos = 0.0;
      int count = 0;
      for (int j = -half_window; j <= half_window; ++j) {
        int idx = static_cast<int>(i) + j;
        if (idx >= 0 && idx < static_cast<int>(rebuilt.size())) {
          sum_x += rebuilt[idx].x;
          sum_y += rebuilt[idx].y;
          sum_sin += std::sin(rebuilt[idx].theta);
          sum_cos += std::cos(rebuilt[idx].theta);
          ++count;
        }
      }
      smoothed[i].x = sum_x / count;
      smoothed[i].y = sum_y / count;
      smoothed[i].theta = std::atan2(sum_sin, sum_cos);
    }
    rebuilt = smoothed;
  }

  return rebuilt;
}

geometry_msgs::msg::Twist LocalPlannerMPC::getControlCmd() {
  std::vector<PathPoint> pts = extractPathPoints(smoothed_local_);
  pts = rebuildReferenceFromEgo(
      pts, current_pose_.pose.position.x, current_pose_.pose.position.y,
      common::yawFromQuaternion(current_pose_.pose.orientation));
  std::vector<double> s = computeArcLength(pts);
  std::vector<PathPoint> x_ref = resamplePath(pts, s);
  std::vector<Control> u_ref = computeReferenceControls(x_ref);

  // 保存重建后的参考轨迹用于可视化发布
  reference_traj_.header.frame_id = "map";
  reference_traj_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  reference_traj_.poses.clear();
  for (const auto& pt : x_ref) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(pt.theta / 2.0);
    pose.pose.orientation.w = std::cos(pt.theta / 2.0);
    reference_traj_.poses.push_back(pose);
  }

  // 设置当前状态
  Eigen::VectorXd x0(nx_);
  x0 << current_pose_.pose.position.x, current_pose_.pose.position.y,
      common::yawFromQuaternion(current_pose_.pose.orientation);
  // OSQP求解MPC
  Control u_opt = solveQP(x_ref, u_ref, x0);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = u_opt.v;
  cmd.angular.z = u_opt.omega;
  return cmd;
}

Control LocalPlannerMPC::solveQP(const std::vector<PathPoint>& x_ref,
                                 const std::vector<Control>& u_ref,
                                 const Eigen::VectorXd& x0) {
  RCLCPP_INFO(logger_, "x0: %f %f %f", x0(0), x0(1), x0(2));
  RCLCPP_INFO(logger_, "x_ref: %f %f %f u_ref %f %f", x_ref[0].x, x_ref[0].y,
              x_ref[0].theta, u_ref[0].v, u_ref[0].omega);
  // 1. 线性化
  /*
    x = [X Y theta] 状态向量
    u = [v omega] 控制向量
    x_{k+1} = f(x_k, u_k) =
      [X_k + dt * V_k * cos(theta_k),
       Y_k + dt * V_k * sin(theta_k),
       theta_k + omega_k * dt]
    x_{k + 1} = A * x_k + B * u_k + d_k
        | 1 0 dt * V_k * -sin(theta_k) |
    A = | 0 1 dt * V_k * cos(theta_k)  |
        | 0 0 1                        |

        | dt * cos(theta_k) 0  |
    B = | dt * sin(theta_k) 0  |
        | 0                 dt |
    d_k 为x_{k + 1}到参考线对应位置的误差
    */
  std::vector<Eigen::MatrixXd> A_vec(N_);
  std::vector<Eigen::MatrixXd> B_vec(N_);
  std::vector<Eigen::MatrixXd> d_vec(N_);

  for (int k = 0; k < N_; ++k) {
    double theta_k = x_ref[k].theta;
    double v_k = u_ref[k].v;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx_, nx_);
    A(0, 2) = dt_ * v_k * -std::sin(theta_k);
    A(1, 2) = dt_ * v_k * std::cos(theta_k);

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx_, nu_);
    B(0, 0) = dt_ * std::cos(theta_k);
    B(1, 0) = dt_ * std::sin(theta_k);
    B(2, 1) = dt_;

    // Eigen::VectorXd xk(nx_), xk1(nx_), uk(nu_);
    // xk << x_ref[k].x, x_ref[k].y, x_ref[k].theta;
    // xk1 << x_ref[k + 1].x, x_ref[k + 1].y, x_ref[k + 1].theta;
    // uk << u_ref[k].v, u_ref[k].omega;

    // Eigen::VectorXd d = xk1 - A * xk - B * uk;

    A_vec[k] = A;
    B_vec[k] = B;
    // d_vec[k] = d;
  }

  // 2. 构造Hessian
  std::vector<Eigen::Triplet<double>> H_triplets;
  for (int k = 0; k <= N_; ++k) {
    int x_idx = k * (nx_ + nu_);
    if (k < N_) {
      for (int i = 0; i < nx_; ++i) {
        for (int j = 0; j < nx_; ++j) {
          if (Q_(i, j) != 0.0) {
            H_triplets.emplace_back(x_idx + i, x_idx + j, Q_(i, j));
          }
        }
      }

      int u_idx = x_idx + nx_;
      for (int i = 0; i < nu_; ++i) {
        for (int j = 0; j < nu_; ++j) {
          if (R_(i, j) != 0.0) {
            H_triplets.emplace_back(u_idx + i, u_idx + j, R_(i, j));
          }
        }
      }
    } else {
      for (int i = 0; i < nx_; ++i) {
        for (int j = 0; j < nx_; ++j) {
          if (P_(i, j) != 0.0) {
            H_triplets.emplace_back(x_idx + i, x_idx + j, P_(i, j));
          }
        }
      }
    }
  }

  Eigen::SparseMatrix<double> H(n_var_, n_var_);
  H.setFromTriplets(H_triplets.begin(), H_triplets.end());

  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_var_);

  // 3. 构造约束 A_constr 和 边界l, u
  std::vector<Eigen::Triplet<double>> A_triplets;
  Eigen::VectorXd l(n_constr_);
  Eigen::VectorXd u(n_constr_);

  // 3.1 初始状态约束
  for (int i = 0; i < nx_; ++i) {
    int row = i;
    int col = i;
    A_triplets.emplace_back(row, col, 1.0);

    double val;
    if (i == 0) {
      val = x0(i) - x_ref[0].x;
    } else if (i == 1) {
      val = x0(i) - x_ref[0].y;
    } else {
      val = common::normalizeAngleDiff(x0(i) - x_ref[0].theta);
    }

    l(row) = val;
    u(row) = val;
  }

  // 3.2 动力学约束
  for (int k = 0; k < N_; ++k) {
    int row = nx_ + k * nx_;
    int xk_col = k * (nx_ + nu_);
    int uk_col = xk_col + nx_;
    int xk1_col = (k + 1) * (nx_ + nu_);

    for (int i = 0; i < nx_; ++i) {
      A_triplets.emplace_back(row + i, xk1_col + i, 1.0);
    }

    for (int i = 0; i < nx_; ++i) {
      for (int j = 0; j < nx_; ++j) {
        if (A_vec[k](i, j) != 0.0) {
          A_triplets.emplace_back(row + i, xk_col + j, -A_vec[k](i, j));
        }
      }
    }

    for (int i = 0; i < nx_; ++i) {
      for (int j = 0; j < nu_; ++j) {
        if (B_vec[k](i, j) != 0.0) {
          A_triplets.emplace_back(row + i, uk_col + j, -B_vec[k](i, j));
        }
      }
    }

    for (int i = 0; i < nx_; ++i) {
      l(row + i) = 0;
      u(row + i) = 0;
    }
  }

  // 3.3 边界约束
  int ineq_start = (N_ + 1) * nx_;
  for (int k = 0; k <= N_; ++k) {
    int x_col = k * (nx_ + nu_);
    int u_col = x_col + nx_;
    int row_x = ineq_start + x_col;
    int row_u = ineq_start + u_col;

    for (int i = 0; i < nx_; ++i) {
      A_triplets.emplace_back(row_x + i, x_col + i, 1.0);
      double xref_val;
      if (i == 0) {
        xref_val = x_ref[k].x;
      } else if (i == 1) {
        xref_val = x_ref[k].y;
      } else {
        // xref_val = x_ref[k].theta;
        l(row_x + i) = x_min_(i);
        u(row_x + i) = x_max_(i);
        continue;
      }

      l(row_x + i) = x_min_(i) - xref_val;
      u(row_x + i) = x_max_(i) - xref_val;
    }

    if (k < N_) {
      for (int i = 0; i < nu_; ++i) {
        A_triplets.emplace_back(row_u + i, u_col + i, 1.0);
        if (i == 0) {
          l(row_u + i) = u_min_(i) - u_ref[k].v;
          u(row_u + i) = u_max_(i) - u_ref[k].v;
        } else {
          l(row_u + i) = u_min_(i) - u_ref[k].omega;
          u(row_u + i) = u_max_(i) - u_ref[k].omega;
        }
      }
    }
  }

  // 4. 构建稀疏矩阵
  Eigen::SparseMatrix<double> A_constr(n_constr_, n_var_);
  A_constr.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // 5. OSQP 求解
  if (!solver_initialized_) {
    solver_.data()->setNumberOfVariables(n_var_);
    solver_.data()->setNumberOfConstraints(n_constr_);
    solver_.data()->setHessianMatrix(H);
    solver_.data()->setGradient(q);
    solver_.data()->setLinearConstraintsMatrix(A_constr);
    solver_.data()->setLowerBound(l);
    solver_.data()->setUpperBound(u);

    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.settings()->setMaxIteration(4000);
    solver_.settings()->setAbsoluteTolerance(1e-3);
    solver_.settings()->setRelativeTolerance(1e-3);

    if (!solver_.initSolver()) {
      RCLCPP_INFO(logger_, "OSQP init failed!");
      return Control{0.0, 0.0};
    }

    solver_initialized_ = true;
  } else {
    solver_.updateHessianMatrix(H);
    solver_.updateGradient(q);
    solver_.updateLinearConstraintsMatrix(A_constr);
    solver_.updateBounds(l, u);
  }

  if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    RCLCPP_INFO(logger_, "OSQP solve failed!");
    return Control{u_ref[0].v, u_ref[0].omega};  // fallback
  }

  // 6. 提取结果
  Eigen::VectorXd sol = solver_.getSolution();

  // 保存由该组控制量前向仿真得到的预测轨迹，用于可视化
  predicted_trajectory_ = predictTrajectory(sol, u_ref, x0);

  int du0_idx = nx_;
  double delta_v = sol(du0_idx);
  double delta_omega = sol(du0_idx + 1);

  Control u_opt;
  u_opt.v = u_ref[0].v + delta_v;
  u_opt.omega = u_ref[0].omega + delta_omega;

  u_opt.v = std::max(u_min_(0), std::min(u_max_(0), u_opt.v));
  u_opt.omega = std::max(u_min_(1), std::min(u_max_(1), u_opt.omega));
  return u_opt;
}

std::vector<PathPoint> LocalPlannerMPC::predictTrajectory(
    const Eigen::VectorXd& sol, const std::vector<Control>& u_ref,
    const Eigen::VectorXd& x0) {
  std::vector<PathPoint> traj;
  traj.reserve(N_ + 1);

  PathPoint pt;
  pt.x = x0(0);
  pt.y = x0(1);
  pt.theta = x0(2);
  traj.push_back(pt);

  for (int k = 0; k < N_; ++k) {
    int u_idx = k * (nx_ + nu_) + nx_;
    Control u;
    u.v = u_ref[k].v + sol(u_idx);
    u.omega = u_ref[k].omega + sol(u_idx + 1);

    // 控制量限幅
    u.v = std::max(u_min_(0), std::min(u_max_(0), u.v));
    u.omega = std::max(u_min_(1), std::min(u_max_(1), u.omega));

    const PathPoint& prev = traj.back();
    PathPoint next;
    next.x = prev.x + dt_ * u.v * std::cos(prev.theta);
    next.y = prev.y + dt_ * u.v * std::sin(prev.theta);
    next.theta = prev.theta + dt_ * u.omega;
    next.theta = common::normalizeAngleDiff(next.theta);
    traj.push_back(next);
  }

  return traj;
}

std::vector<geometry_msgs::msg::Point> LocalPlannerMPC::computeVehicleBox(
    double x, double y, double theta) {
  const double half_l = common::constants::vehicle_length / 2.0;
  const double half_w = common::constants::vehicle_width / 2.0;

  // 车辆坐标系下的角点：前左、前右、后右、后左
  std::vector<std::pair<double, double>> local_corners = {{half_l, half_w},
                                                          {half_l, -half_w},
                                                          {-half_l, -half_w},
                                                          {-half_l, half_w}};

  std::vector<geometry_msgs::msg::Point> corners;
  corners.reserve(local_corners.size());
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  for (const auto& local : local_corners) {
    geometry_msgs::msg::Point p;
    p.x = x + local.first * c - local.second * s;
    p.y = y + local.first * s + local.second * c;
    p.z = 0.0;
    corners.push_back(p);
  }
  return corners;
}

void LocalPlannerMPC::visualizeSampledTrajectories(
    const std::string& frame_id) {
  auto& vis = visualization::VisualizationManager::Instance();
  vis.ClearNamespace("mpc_predicted_boxes");

  if (predicted_trajectory_.empty()) {
    return;
  }

  // 每隔一个预测步长显示一个box，避免过于密集
  const int step = 2;
  const float alpha = 0.8f;
  visualization::Color color(0.0f, 0.8f, 1.0f, alpha);

  int id = 0;
  for (size_t i = 0; i < predicted_trajectory_.size(); i += step) {
    const auto& pt = predicted_trajectory_[i];
    auto box = computeVehicleBox(pt.x, pt.y, pt.theta);
    vis.AddPolygon("mpc_predicted_boxes", id++, box, color, 0.02f, frame_id);
  }
}

}  // namespace local_planner
