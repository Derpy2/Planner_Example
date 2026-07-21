#include "planner/global_planner/global_planner_base.h"

#include <cmath>
#include <nav_msgs/msg/path.hpp>

#include "planner/global_planner/complete_cover_path/bcd.h"

namespace global_planner {

void GlobalPlannerBase::setCoverArea(
    const common::Polygon2D& boundary,
    const std::vector<common::Polygon2D>& obstacles) {
  cover_boundary_ = boundary;
  cover_obstacles_ = obstacles;
  has_cover_area_ = true;
}

void GlobalPlannerBase::setSweepParams(double offset,
                                       const common::Node2D& dir) {
  cover_offset_ = offset;
  cover_dir_ = dir;
}

nav_msgs::msg::Path GlobalPlannerBase::getGlobalCoverPath() {
  nav_msgs::msg::Path path;
  if (!has_cover_area_) {
    RCLCPP_WARN(logger_, "Cover area not set, call setCoverArea first.");
    return path;
  }

  complete_cover_path::BCDDecomposer decomposer;
  std::vector<common::Polygon2D> cells =
      decomposer.decompose(cover_boundary_, cover_obstacles_);
  std::vector<common::Pose2D> cover_waypoints =
      decomposer.generateGlobalCoverPath(cells, cover_offset_, cover_dir_);

  if (cover_waypoints.empty()) {
    RCLCPP_WARN(logger_, "Failed to generate coverage path.");
    return path;
  }

  return convertPose2DToPath(cover_waypoints);
}

nav_msgs::msg::Path GlobalPlannerBase::generateCompleteCoverPath() {
  nav_msgs::msg::Path complete_path;
  if (!has_cover_area_) {
    RCLCPP_WARN(logger_, "Cover area not set, call setCoverArea first.");
    return complete_path;
  }

  // 1. 生成纯覆盖路径
  complete_cover_path::BCDDecomposer decomposer;
  std::vector<common::Polygon2D> cells =
      decomposer.decompose(cover_boundary_, cover_obstacles_);
  std::vector<common::Pose2D> cover_waypoints =
      decomposer.generateGlobalCoverPath(cells, cover_offset_, cover_dir_);

  if (cover_waypoints.empty()) {
    RCLCPP_WARN(logger_, "Failed to generate coverage path.");
    return complete_path;
  }

  nav_msgs::msg::Path cover_path = convertPose2DToPath(cover_waypoints);

  // 2. 使用子类方法规划 自车位置 -> coverage起点
  const common::Pose2D& cover_start = cover_waypoints.front();
  geometry_msgs::msg::Pose cover_start_pose;
  cover_start_pose.position.x = cover_start.x;
  cover_start_pose.position.y = cover_start.y;
  cover_start_pose.orientation.z = std::sin(cover_start.theta / 2.0);
  cover_start_pose.orientation.w = std::cos(cover_start.theta / 2.0);

  nav_msgs::msg::Path approach_path =
      planBetween(start_pose_, cover_start_pose);

  // 3. 拼接路径
  if (!approach_path.poses.empty()) {
    complete_path.poses.insert(complete_path.poses.end(),
                               approach_path.poses.begin(),
                               approach_path.poses.end());

    // 避免连接点重复
    const geometry_msgs::msg::Point& last_approach =
        approach_path.poses.back().pose.position;
    const geometry_msgs::msg::Point& first_cover =
        cover_path.poses.front().pose.position;
    const double dx = last_approach.x - first_cover.x;
    const double dy = last_approach.y - first_cover.y;
    if (std::hypot(dx, dy) > 1e-3) {
      complete_path.poses.insert(complete_path.poses.end(),
                                 cover_path.poses.begin(),
                                 cover_path.poses.end());
    } else {
      complete_path.poses.insert(complete_path.poses.end(),
                                 cover_path.poses.begin() + 1,
                                 cover_path.poses.end());
    }
  } else {
    // 自车->coverage起点 规划失败，按用户选择仅返回 coverage 路径
    RCLCPP_WARN(logger_,
                "Approach path from ego to coverage start failed, "
                "returning coverage path only.");
    complete_path.poses.insert(complete_path.poses.end(),
                               cover_path.poses.begin(),
                               cover_path.poses.end());
  }

  return complete_path;
}

nav_msgs::msg::Path GlobalPlannerBase::planBetween(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& goal) {
  // 临时替换 start_pose_ / goal_pose_，调用子类 searchPath()
  const geometry_msgs::msg::Pose original_start = start_pose_;
  const geometry_msgs::msg::Pose original_goal = goal_pose_;
  start_pose_ = start;
  goal_pose_ = goal;
  nav_msgs::msg::Path path = searchPath();
  start_pose_ = original_start;
  goal_pose_ = original_goal;
  return path;
}

nav_msgs::msg::Path GlobalPlannerBase::convertPose2DToPath(
    const std::vector<common::Pose2D>& waypoints) {
  nav_msgs::msg::Path path;
  path.poses.reserve(waypoints.size());
  for (const auto& wp : waypoints) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = wp.x;
    p.pose.position.y = wp.y;
    p.pose.position.z = 0.0;
    p.pose.orientation.z = std::sin(wp.theta / 2.0);
    p.pose.orientation.w = std::cos(wp.theta / 2.0);
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace global_planner
