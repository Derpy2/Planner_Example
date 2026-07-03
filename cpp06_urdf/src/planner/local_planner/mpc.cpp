#include "planner/local_planner/mpc.h"

namespace local_planner {
void LocalPlannerMPC::Init(const int nx, const int nu, const int N, double dt,
                           double v_ref) {
  nx_ = nx;
  nu_ = nu;
  N_ = N;
  dt_ = dt;
  v_ref_ = v_ref;
  n_var_ = (N + 1) * nx + N * nu;
  n_constr_ = N * nx + n_var_;
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

geometry_msgs::msg::Twist LocalPlannerMPC::getControlCmd() {
  std::vector<PathPoint> pts = extractPathPoints(smoothed_local_);
  std::vector<double> s = computeArcLength(pts);
  std::vector<PathPoint> x_ref = resamplePath(pts, s);
  std::vector<Control> u_ref = computeReferenceControls(x_ref);

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
  // 1. 线性化
  std::vector<Eigen::MatrixXd> A_vec(N_);
  std::vector<Eigen::MatrixXd> B_vec(N_);
  std::vector<Eigen::MatrixXd> d_vec(N_);

  for (int k = 0; k < N_; ++k) {
    double theta_k = x_ref[k].theta;
    double v_k = u_ref[k].v;

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(nx_, nx_);
    A(0, 2) = -dt_ * v_k * std::sin(theta_k);
    A(1, 2) = dt_ * v_k * std::cos(theta_k);

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx_, nu_);
    B(0, 0) = dt_ * std::cos(theta_k);
    B(1, 0) = dt_ * std::sin(theta_k);
    B(2, 1) = dt_;

    Eigen::VectorXd xk(nx_), xk1(nx_), uk(nu_);
    xk << x_ref[k].x, x_ref[k].y, x_ref[k].theta;
    xk1 << x_ref[k + 1].x, x_ref[k + 1].y, x_ref[k + 1].theta;
    uk << u_ref[k].v, u_ref[k].omega;

    Eigen::VectorXd d = xk1 - A * xk - B * uk;

    A_vec[k] = A;
    B_vec[k] = B;
    d_vec[k] = d;
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
      l(row + i) = d_vec[k](i);
      u(row + i) = d_vec[k](i);
    }
  }

  // 3.3 边界约束
  int ineq_start = N_ * nx_;
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
        xref_val = x_ref[k].theta;
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
      std::cerr << "OSQP init failed!" << std::endl;
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
    std::cerr << "OSQP solve failed!" << std::endl;
    return Control{u_ref[0].v, u_ref[0].omega};  // fallback
  }

  // 6. 提取结果
  Eigen::VectorXd sol = solver_.getSolution();

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

}  // namespace local_planner
