#include "planner/local_planner/pure_pursuit.h"

namespace local_planner {

geometry_msgs::msg::Twist PurePursuit::getControlCmd() {
  double current_x, current_y;
  current_x = current_pose_.pose.position.x;
  current_y = current_pose_.pose.position.y;
  // 纯追踪 (Pure Pursuit) 找前视点
  int target_idx = -1;
  for (size_t i = 0; i < smoothed_local_.poses.size(); ++i) {
    double dx = smoothed_local_.poses[i].pose.position.x - current_x;
    double dy = smoothed_local_.poses[i].pose.position.y - current_y;
    double dist = std::hypot(dx, dy);
    if (dist >= constants::lookaheadDistance) {
      target_idx = i;
      break;
    }
  }
  if (target_idx == -1 && 1 < smoothed_local_.poses.size()) {
    target_idx = smoothed_local_.poses.size() - 1;
  }

  double target_x = smoothed_local_.poses[target_idx].pose.position.x;
  double target_y = smoothed_local_.poses[target_idx].pose.position.y;
  double dx = target_x - current_x;
  double dy = target_y - current_y;
  double target_yaw = std::atan2(dy, dx);
  double alpha =
      target_yaw - common::yawFromQuaternion(current_pose_.pose.orientation);
  // 归一化到 [-pi, pi]
  while (alpha > M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  double L = std::hypot(dx, dy);
  if (L < 1e-3) L = constants::lookaheadDistance;

  // 曲率控制
  double v = constants::maxLinearSpeed;
  // 大角度时减速
  v *= std::max(0.0, std::cos(alpha));
  if (v < 0.05) v = 0.05;

  double curvature = 2.0 * std::sin(alpha) / L;
  double omega = v * curvature;
  if (omega > constants::maxAngularSpeed) {
    omega = constants::maxAngularSpeed;
  }
  if (omega < -constants::maxAngularSpeed) {
    omega = -constants::maxAngularSpeed;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = omega;

  return cmd;
}

}  // namespace local_planner