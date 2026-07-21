#pragma once

#include <algorithm>
#include <vector>

#include "common/node2d.h"

namespace global_planner {
namespace complete_cover_path {

using namespace common;

class Sweep {
 public:
  static std::vector<Node2D> sortPointsToLine(const Polygon2D& poly,
                                              const Node2D& dir);

  // 对牛耕法BCD生成的每个cell生成cell内部的扫描线全覆盖路径，存放在waypoints里面，
  static bool computeSweep(const Polygon2D& poly, const double offset,
                           const Node2D& dir, std::vector<Pose2D>* waypoints);
};

}  // namespace complete_cover_path
}  // namespace global_planner
