#pragma once

#include <cmath>
#include <vector>

namespace common {

namespace {
constexpr double epsilon = 1e-8;
}

// ===========Data=============
struct Node2D {
  double x, y;
  Node2D(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
  bool operator<(const Node2D& other) const {
    if (std::fabs(x - other.x) < epsilon) {
      return y < other.y;
    }
    return x < other.x;
  }

  bool operator==(const Node2D& other) const {
    return std::fabs(x - other.x) < epsilon && std::fabs(y - other.y) < epsilon;
  }
};

struct Segment2D {
  Node2D p0, p1;
  Segment2D() : p0(Node2D(0.0, 0.0)), p1(Node2D(0.0, 0.0)) {}
  Segment2D(Node2D a, Node2D b) : p0(a), p1(b) {}

  bool operator==(const Segment2D& other) {
    return p0 == other.p0 && p1 == other.p1;
  }

  Segment2D opposite() { return Segment2D(p1, p0); }
};

struct Line2D {
  // a*x + b*y + c = 0
  double a, b, c;
  Line2D(const Node2D& point, const Node2D& dir) {
    a = -dir.y;
    b = dir.x;
    c = -(a * point.x + b * point.y);
  }
};

using Polygon2D = std::vector<Node2D>;

struct Pose2D : public Node2D {
  double theta;
  Pose2D(double x_, double y_, double theta_) : Node2D(x_, y_), theta(theta_) {}
};

// ===========Function=============
static inline double distance(const Node2D& a, const Node2D& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

static inline double cross(const Node2D& a, const Node2D& b, const Node2D& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static inline bool colinear(const Node2D& a, const Node2D& b, const Node2D& c) {
  double dx1 = b.x - a.x;
  double dx2 = c.x - a.x;
  double dy1 = b.y - a.y;
  double dy2 = c.y - a.y;

  double cr = dx1 * dy2 - dx2 * dy1;
  return std::fabs(cr) < epsilon;
}

static inline bool isPointInPolygon(const Node2D& point,
                                    const Polygon2D& polygon) {
  // 射线法判断点在多边形内(在边上属于内部)
  const int pt_size = polygon.size();
  bool inside = false;
  for (int i = 0; i < pt_size; ++i) {
    const Node2D& a = polygon[i];
    const Node2D& b = polygon[(i + 1) % pt_size];

    // 射线检测
    if ((point.y > a.y) != (point.y > b.y) &&
        (point.x < (point.y - b.y) / (a.y - b.y) * (a.x - b.x) + b.x)) {
      inside = !inside;
    }
  }
  return inside;
}

static inline Node2D normalizeDir(const Node2D& dir) {
  double len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
  return Node2D(dir.x / len, dir.y / len);
}

// static double dot(const Node2D& a, const Node2D& b) {
//   return a.x * b.x + a.y * b.y;
// }
}  // namespace common