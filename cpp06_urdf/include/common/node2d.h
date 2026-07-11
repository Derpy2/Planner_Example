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
  Segment2D(Node2D a, Node2D b) : p0(a), p1(b) {}
};

using Polygon2D = std::vector<Node2D>;

struct Pose2D : public Node2D {
  double theta;
  Pose2D(double x_, double y_, double theta_) : Node2D(x_, y_), theta(theta_) {}
};

// ===========Function=============
static inline double cross(const Node2D& a, const Node2D& b, const Node2D& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// static double dot(const Node2D& a, const Node2D& b) {
//   return a.x * b.x + a.y * b.y;
// }
}  // namespace common