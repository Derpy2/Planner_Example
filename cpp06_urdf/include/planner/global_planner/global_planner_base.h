#pragma once

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/node2d.h"
#include "map/static_map.h"

namespace global_planner {
class GlobalPlannerBase {
 public:
  GlobalPlannerBase(std::shared_ptr<map::StaticMap> map,
                    const rclcpp::Logger logger)
      : map_(map), logger_(logger) {}
  virtual ~GlobalPlannerBase() = default;

  virtual void setStartPose(const geometry_msgs::msg::Pose& start_pose) {
    start_pose_ = start_pose;
  }

  virtual void setGoalPose(const geometry_msgs::msg::Pose& goal_pose) {
    goal_pose_ = goal_pose;
  }

  // 设置覆盖区域（使用世界坐标，边界多边形与障碍物多边形列表）
  void setCoverArea(const common::Polygon2D& boundary,
                    const std::vector<common::Polygon2D>& obstacles);

  // 设置扫描参数：扫描线间距与扫描方向
  void setSweepParams(double offset, const common::Node2D& dir);

  virtual nav_msgs::msg::Path searchPath() = 0;

  // 生成纯覆盖路径（BCD + 牛耕法 + TSP）
  nav_msgs::msg::Path getGlobalCoverPath();

  // 生成完整路径：自车位置 -> coverage起点 -> coverage路径 -> coverage终点
  // 其中 自车位置->coverage起点 使用子类自己的 searchPath() 方法
  nav_msgs::msg::Path generateCompleteCoverPath();

 protected:
  // 使用子类 searchPath() 规划 start 到 goal 的路径
  nav_msgs::msg::Path planBetween(const geometry_msgs::msg::Pose& start,
                                  const geometry_msgs::msg::Pose& goal);

  // 将 Pose2D 序列转换为 nav_msgs::msg::Path
  static nav_msgs::msg::Path convertPose2DToPath(
      const std::vector<common::Pose2D>& waypoints);

 protected:
  std::shared_ptr<map::StaticMap> map_ = nullptr;
  const rclcpp::Logger logger_;

  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  // 覆盖区域与扫描参数
  common::Polygon2D cover_boundary_;
  std::vector<common::Polygon2D> cover_obstacles_;
  double cover_offset_ = 0.6;
  common::Node2D cover_dir_;
  bool has_cover_area_ = false;
};
}  // namespace global_planner
