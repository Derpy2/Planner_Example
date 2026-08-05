#pragma once

#include <nav_msgs/msg/path.hpp>

#include <vector>

#include "common/node2d.h"

namespace global_planner {
namespace complete_cover_path {

struct BoundaryPathOptions {
  double offset = 0.0;       // 与障碍物边缘的安全距离，>0 向自由空间内缩
  double arc_radius = 0.0;   // 拐角圆弧半径，<=0 不插入圆弧，与 offset 解耦
  bool clockwise = true;     // 每个障碍物边界的绕行方向
  int arc_samples = 8;       // 每个拐角圆弧的采样点数
};

class BoundaryPathGenerator {
 public:
  // 为每个障碍物生成一条独立的开放延边路径，返回每条路径的 Pose 序列
  static std::vector<std::vector<common::Pose2D>> generateObstacleBoundaries(
      const common::Polygon2D& boundary,
      const std::vector<common::Polygon2D>& obstacles,
      const BoundaryPathOptions& options);

  // 对单个障碍物多边形生成带 offset + 圆弧平滑的边界路径（首尾闭合以完整覆盖）
  static std::vector<common::Pose2D> traceSingleObstacleBoundary(
      const common::Polygon2D& obstacle,
      const BoundaryPathOptions& options);

  // 生成由所有障碍物延边路径拼接而成的全局路线（nav_msgs::msg::Path 格式）
  // 使用最近邻启发式确定障碍物访问顺序：从自车起点出发，每次选择最近
  // 的未访问障碍物，并在相邻障碍物路径之间插入直线过渡段
  static nav_msgs::msg::Path generateGlobalBoundaryPath(
      const common::Polygon2D& boundary,
      const std::vector<common::Polygon2D>& obstacles,
      const BoundaryPathOptions& options,
      const geometry_msgs::msg::Pose& start_pose = geometry_msgs::msg::Pose());

 private:
  // 对多边形做内缩 offset（输入为 CW 多边形，offset > 0 向自由空间方向偏移）
  static common::Polygon2D insetPolygon(const common::Polygon2D& polygon,
                                        double offset);

  // 在 path 末尾追加 corner 处的圆弧段
  static void appendCornerArc(std::vector<common::Pose2D>& path,
                              const common::Node2D& prev,
                              const common::Node2D& corner,
                              const common::Node2D& next,
                              const BoundaryPathOptions& options);

  // 删除共线/重合点，返回是否 >= 3 个顶点
  static bool cleanupPolygon(common::Polygon2D* poly);

  // 判断多边形是否为顺时针（有向面积 < 0）
  static bool isClockwise(const common::Polygon2D& polygon);
};

}  // namespace complete_cover_path
}  // namespace global_planner
