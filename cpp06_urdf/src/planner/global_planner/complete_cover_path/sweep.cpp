#include "planner/global_planner/complete_cover_path/sweep.h"

namespace global_planner {
namespace complete_cover_path {

namespace {
constexpr double epsilon = 1e-8;
}

std::vector<Node2D> Sweep::sortPointsToLine(const Polygon2D& poly,
                                            const Node2D& dir) {
  // dir 与扫描线同向，dir为单位向量
  std::vector<Node2D> points = poly;

  std::sort(points.begin(), points.end(),
            [&dir](const Node2D& a, const Node2D& b) -> bool {
              return (a.x * dir.y - a.y * dir.x) < (b.x * dir.y - b.y * dir.x);
            });

  return points;
}

// 对牛耕法BCD生成的每个cell生成cell内部的扫描线全覆盖路径，存放在waypoints里面，
bool Sweep::computeSweep(const Polygon2D& poly, const double offset,
                         const Node2D& dir, std::vector<Pose2D>* waypoints) {
  waypoints->clear();
  if (poly.size() < 3 || offset <= 1e-8) {
    return false;
  }

  const Node2D unit_dir = normalizeDir(dir);
  const Node2D obstacle_gap = Node2D(unit_dir.x * 0.4, unit_dir.y * 0.4);
  // 沿扫描线法向量排列，由近到远
  std::vector<Node2D> sorted_pts = sortPointsToLine(poly, dir);
  // 获得cell整体offset
  double cell_offset =
      (sorted_pts.back().x * unit_dir.y - sorted_pts.back().y * unit_dir.x) -
      (sorted_pts.front().x * unit_dir.y - sorted_pts.front().y * unit_dir.x);
  if (cell_offset <= 1e-8) {
    return false;
  }

  bool counter_clockwise = true;

  // 获得dir法向量（沿投影递增方向，即从sorted_pts.front()指向sorted_pts.back()）
  Node2D nor_dir = Node2D(unit_dir.y, -unit_dir.x);

  // 从距离边界 offset/2 处开始第一条扫描线，保证覆盖到cell边界
  double current_offset = offset / 2.0;
  while (current_offset <= cell_offset - offset / 2.0 + 1e-8) {
    Node2D base(sorted_pts[0].x + current_offset * nor_dir.x,
                sorted_pts[0].y + current_offset * nor_dir.y);
    Line2D sweep_line(base, unit_dir);

    // 求扫描线与polygon各边的交点
    std::vector<Node2D> intersections;
    const int pt_size = static_cast<int>(poly.size());
    for (int i = 0; i < pt_size; ++i) {
      const Node2D& p0 = poly[i];
      const Node2D& p1 = poly[(i + 1) % pt_size];

      double d0 = sweep_line.a * p0.x + sweep_line.b * p0.y + sweep_line.c;
      double d1 = sweep_line.a * p1.x + sweep_line.b * p1.y + sweep_line.c;

      // 边与扫描线平行或共线
      if (std::fabs(d0 - d1) < epsilon) {
        continue;
      }

      double t = d0 / (d0 - d1);
      if (t >= -epsilon && t <= 1.0 + epsilon) {
        intersections.emplace_back(p0.x + t * (p1.x - p0.x),
                                   p0.y + t * (p1.y - p0.y));
      }
    }

    if (intersections.size() < 2) {
      current_offset += offset;
      counter_clockwise = !counter_clockwise;
      continue;
    }

    // 沿unit_dir方向排序交点
    std::sort(intersections.begin(), intersections.end(),
              [&unit_dir](const Node2D& a, const Node2D& b) -> bool {
                return (a.x * unit_dir.x + a.y * unit_dir.y) <
                       (b.x * unit_dir.x + b.y * unit_dir.y);
              });

    // 去除重复交点
    // std::vector<Node2D> unique_intersections;
    // for (const auto& pt : intersections) {
    //   if (unique_intersections.empty() ||
    //       std::fabs(unique_intersections.back().x - pt.x) > 1e-8 ||
    //       std::fabs(unique_intersections.back().y - pt.y) > 1e-8) {
    //     unique_intersections.push_back(pt);
    //   }
    // }

    // 每两个交点构成一段在cell内部的扫描线段
    std::vector<std::pair<Node2D, Node2D>> segments;
    for (size_t i = 0; i + 1 < intersections.size(); i += 2) {
      // 与障碍物或者边界保持一定距离

      segments.emplace_back(intersections[i] + obstacle_gap,
                            intersections[i + 1] - obstacle_gap);
    }

    if (counter_clockwise) {
      // 沿unit_dir正向遍历
      double theta = std::atan2(unit_dir.y, unit_dir.x);
      for (const auto& seg : segments) {
        waypoints->emplace_back(seg.first.x, seg.first.y, theta);
        waypoints->emplace_back(seg.second.x, seg.second.y, theta);
      }
    } else {
      // 沿unit_dir反向遍历，保持蛇形连续
      double theta = std::atan2(-unit_dir.y, -unit_dir.x);
      for (auto it = segments.rbegin(); it != segments.rend(); ++it) {
        waypoints->emplace_back(it->second.x, it->second.y, theta);
        waypoints->emplace_back(it->first.x, it->first.y, theta);
      }
    }

    current_offset += offset;
    counter_clockwise = !counter_clockwise;
  }

  return !waypoints->empty();
}

}  // namespace complete_cover_path
}  // namespace global_planner