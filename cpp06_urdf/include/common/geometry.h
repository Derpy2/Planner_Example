#include <vector>

#include "common/node3d.h"

namespace common {

namespace {
constexpr double eplison = 1e-10;
}

static inline bool isPointInPolygon(const Node3D& point,
                                    const std::vector<Node3D>& polygon) {
  // 射线法判断点在多边形内
  const int pt_size = polygon.size();
  bool inside = false;
  for (int i = 0; i < pt_size; ++i) {
    const Node3D& a = polygon[i];
    const Node3D& b = polygon[(i + 1) % pt_size];

    // 射线检测
    if ((point.getY() > a.getY()) != (point.getY() > b.getY()) &&
        (point.getX() <
         (point.getY() - b.getY()) /
             (a.getY() - b.getY() * (a.getX() - b.getX()) + b.getX()))) {
      inside = !inside;
    }
  }
  return inside;
}

static inline double pointToSegmentDistance(const Node3D& point,
                                            const Node3D& seg_start,
                                            const Node3D& seg_end) {
  double sx = seg_end.getX() - seg_start.getX();
  double sy = seg_end.getY() - seg_start.getY();

  double seg_len_sq = sx * sx + sy * sy;

  double px = point.getX() - seg_start.getX();
  double py = point.getY() - seg_start.getY();

  if (seg_len_sq < eplison) {
    return std::sqrt(px * px + py * py);
  }

  double t = (px * sx + py * sy) / seg_len_sq;

  t = std::max(0.0, std::min(1.0, t));

  double cloest_x = seg_start.getX() + t * sx;
  double cloest_y = seg_start.getY() + t * sy;

  double dx = point.getX() - cloest_x;
  double dy = point.getY() - cloest_y;
  return std::sqrt(dx * dx + dy * dy);
}

static inline double pointToPolygonDistance(
    const Node3D& point, const std::vector<Node3D>& polygon) {
  if (isPointInPolygon(point, polygon)) {
    return 0.0;
  }
  int n = polygon.size();
  double min_dist = std::numeric_limits<double>::infinity();

  for (int i = 0; i < n; ++i) {
    const Node3D& seg_start = polygon[i];
    const Node3D& seg_end = polygon[(i + 1) % n];
    double dist = pointToSegmentDistance(point, seg_start, seg_end);
    min_dist = std::min(min_dist, dist);
  }

  return min_dist;
}

}  // namespace common