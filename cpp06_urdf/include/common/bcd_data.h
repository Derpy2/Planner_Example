#pragma once

#include "common/node2d.h"

namespace common {
// 临界点类型
enum EventType {
  NONE = 0,
  SPLIT = 1,
  MERGE = 2,
  NORMAL_UP = 3,
  NORMAL_DOWN = 4
};

// 临界点
struct EventPoint {
  Node2D pt;
  EventType type;
  std::vector<Segment2D> adj_segs;
  EventPoint(Node2D p, EventType t) : pt(p), type(t) {}
  bool operator<(const EventPoint& other) const { return pt < other.pt; }
};

// 扫描线Y区间
struct YInterval {
  double y_low, y_high;
  int id;
  YInterval(double l, double h, int i) : y_low(l), y_high(h), id(i) {}
};

// 活动边AEL：扫描线当前相交的多边形边
struct ActiveEdge {
  Segment2D seg;
  double x_cross;
  double y_cross;
  ActiveEdge(Segment2D s, double x, double y)
      : seg(s), x_cross(x), y_cross(y) {}
};

struct BCDNode2D {
  Node2D pt;
  bool is_boundary;
  int polygon_idx;
  int obs_idx;

  BCDNode2D(Node2D point, bool is_bound, int idx, int obs_index)
      : pt(point),
        is_boundary(is_bound),
        polygon_idx(idx),
        obs_idx(obs_index) {}
};

// ===============Function ================
static inline bool segmentVerticalIntersect(const Segment2D& seg, double xs,
                                            double& out_y) {
  double x0 = seg.p0.x;
  double x1 = seg.p1.x;
  if ((xs - x0) * (xs - x1) > epsilon) {
    return false;
  }

  if (std::fabs(x1 - x0) < epsilon) {
    return false;
  }
  double t = (xs - x0) / (x1 - x0);
  out_y = seg.p0.y + t * (seg.p1.y - seg.p0.y);
  return true;
}

}  // namespace common