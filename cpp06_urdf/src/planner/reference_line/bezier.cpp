#include "../include/planner/reference_line/bezier.h"

#include <Eigen/Geometry>
#include <iostream>

namespace reference_line {

using namespace Eigen;

nav_msgs::msg::Path BezierReferenceLine::smoothPath(
    const nav_msgs::msg::Path& path) {
  nav_msgs::msg::Path smoothed_path;
  std::vector<geometry_msgs::msg::PoseStamped> control_points = path.poses;
  if (k_ == 3) {
    smoothed_path.poses = GetBezierCurve(path.poses);
  } else {
    std::cout << "Unsupported Bezier curve order: " << k_ << std::endl;
  }
  return smoothed_path;
}

std::vector<geometry_msgs::msg::PoseStamped>
BezierReferenceLine::GetBezierCurve(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points) {
  std::vector<geometry_msgs::msg::PoseStamped> bezier_curve;
  const size_t control_points_size = control_points.size();
  bezier_curve.emplace_back(control_points.front());
  if (k_ == 3) {
    // 3阶约束位置和速度连续
    for (size_t i = 0; i < control_points_size - 1; ++i) {
      Vector2d w0 = Vector2d(control_points[i].pose.position.x,
                             control_points[i].pose.position.y);
      Vector2d w1 = Vector2d(control_points[i + 1].pose.position.x,
                             control_points[i + 1].pose.position.y);
      Vector2d offset = (w1 - w0) * init_scale_;
      Vector2d p1, p2;
      if (i == 0) {
        p1 = w0 + offset;
        p2 = w1 - offset;
      } else {
        const geometry_msgs::msg::PoseStamped& last_pose =
            *(bezier_curve.end() - 1);
        const geometry_msgs::msg::PoseStamped& sec_last_pose =
            *(bezier_curve.end() - 2);
        Vector2d a2 =
            Vector2d(last_pose.pose.position.x, last_pose.pose.position.y);
        Vector2d a1 = Vector2d(sec_last_pose.pose.position.x,
                               sec_last_pose.pose.position.y);
        // C1 连续 B1 = 2 * A2 - A1
        p1 = 2 * a2 - a1;
        p2 = w1 - offset;
      }
      geometry_msgs::msg::PoseStamped new_pose;
      new_pose.pose.position.x = p1.x();
      new_pose.pose.position.y = p1.y();
      bezier_curve.emplace_back(new_pose);
      new_pose.pose.position.x = p2.x();
      new_pose.pose.position.y = p2.y();
      bezier_curve.emplace_back(new_pose);
      new_pose.pose.position.x = w1.x();
      new_pose.pose.position.y = w1.y();
      bezier_curve.emplace_back(new_pose);
    }
  }
  return bezier_curve;
}
}  // namespace reference_line