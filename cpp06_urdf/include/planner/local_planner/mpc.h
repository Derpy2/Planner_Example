#pragma once

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <vector>

#include "common/util.h"
#include "local_planner_base.h"
#include "visualization/visualization_manager.h"

namespace {
constexpr double epsilon = 1e-6;
}

namespace local_planner {

using namespace Eigen;

struct PathPoint {
  double x, y, theta;
};

struct Control {
  double v, omega;
};

class LocalPlannerMPC : public LocalPlannerBase {
 public:
  LocalPlannerMPC(std::shared_ptr<map::StaticMap> map,
                  const rclcpp::Logger logger)
      : LocalPlannerBase(map, logger) {}

  void init() override { init(3, 2, 20, 0.05, 0.3); }

  void init(const int nx, const int nu, const int N, double dt, double v_ref);

  geometry_msgs::msg::Twist getControlCmd() override;

  void visualizeSampledTrajectories(const std::string& frame_id = "map") override;

 private:
  // 1. 提取参考线轨迹
  std::vector<PathPoint> extractPathPoints(const nav_msgs::msg::Path& path);

  // 2. 计算累计弧长
  std::vector<double> computeArcLength(const std::vector<PathPoint>& pts);
  // 3. 按时间步长重采样
  std::vector<PathPoint> resamplePath(const std::vector<PathPoint>& pts,
                                      const std::vector<double>& s);
  // 4. 计算参考线控制量
  std::vector<Control> computeReferenceControls(
      const std::vector<PathPoint>& ref_traj);

  Control solveQP(const std::vector<PathPoint>& x_ref,
                  const std::vector<Control>& u_ref, const Eigen::VectorXd& x0);

  // 根据优化结果提取完整控制序列，并前向仿真得到预测位姿轨迹
  std::vector<PathPoint> predictTrajectory(
      const Eigen::VectorXd& sol, const std::vector<Control>& u_ref,
      const Eigen::VectorXd& x0);

  // 根据车辆中心位姿生成自车box角点（逆时针）
  std::vector<geometry_msgs::msg::Point> computeVehicleBox(double x, double y,
                                                           double theta);

 private:
  int nx_ = 3;          // 状态维度
  int nu_ = 2;          // 控制维度
  int N_ = 20;          // 预测时域
  int n_var_ = 0;       // 优化变量总数
  int n_constr_ = 0;    // 等式约束 + 边界约束
  double v_ref_ = 0.3;  // 参考速度
  double dt_ = 0.05;    // 采样时间步长

  // 权重矩阵
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;

  // 约束
  Eigen::VectorXd x_min_, x_max_;
  Eigen::VectorXd u_min_, u_max_;

  // OSQP求解器
  OsqpEigen::Solver solver_;
  bool solver_initialized_;

  // 最近一次MPC求解得到的最优预测轨迹（用于可视化）
  std::vector<PathPoint> predicted_trajectory_;
};

}  // namespace local_planner
