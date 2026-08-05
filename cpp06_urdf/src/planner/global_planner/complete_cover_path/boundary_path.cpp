#include "planner/global_planner/complete_cover_path/boundary_path.h"

#include <algorithm>
#include <cmath>

namespace global_planner {
namespace complete_cover_path {

namespace {
constexpr double kEpsilon = 1e-8;

// === geometry helpers ===

double signedPolygonArea(const common::Polygon2D& polygon) {
  double area = 0.0;
  const int n = static_cast<int>(polygon.size());
  if (n < 3) return 0.0;
  for (int i = 0; i < n; ++i) {
    const common::Node2D& a = polygon[i];
    const common::Node2D& b = polygon[(i + 1) % n];
    area += a.x * b.y - b.x * a.y;
  }
  return area * 0.5;
}

common::Node2D vecMul(const common::Node2D& v, double s) {
  return common::Node2D(v.x * s, v.y * s);
}

common::Node2D vecAdd(const common::Node2D& a, const common::Node2D& b) {
  return common::Node2D(a.x + b.x, a.y + b.y);
}

common::Node2D vecSub(const common::Node2D& a, const common::Node2D& b) {
  return common::Node2D(a.x - b.x, a.y - b.y);
}

double cross2D(const common::Node2D& a, const common::Node2D& b) {
  return a.x * b.y - a.y * b.x;
}

}  // namespace

// ========== BoundaryPathGenerator static methods ==========

bool BoundaryPathGenerator::isClockwise(const common::Polygon2D& polygon) {
  return signedPolygonArea(polygon) < -kEpsilon;
}

bool BoundaryPathGenerator::cleanupPolygon(common::Polygon2D* poly) {
  // remove duplicate consecutive vertices
  bool erased = true;
  while (erased) {
    erased = false;
    int n = static_cast<int>(poly->size());
    for (int i = 0; i < n; ++i) {
      if ((*poly)[i] == (*poly)[(i + 1) % n]) {
        poly->erase(poly->begin() + i);
        erased = true;
        break;
      }
    }
  }
  if (poly->size() < 3) return false;

  // remove collinear vertices
  int n = static_cast<int>(poly->size());
  for (int i = 0; i < n; ++i) {
    const common::Node2D& prev = (*poly)[(i - 1 + n) % n];
    const common::Node2D& curr = (*poly)[i];
    const common::Node2D& next = (*poly)[(i + 1) % n];
    if (common::colinear(prev, curr, next)) {
      poly->erase(poly->begin() + i);
      --i;
      --n;
    }
  }
  return poly->size() >= 3;
}

common::Polygon2D BoundaryPathGenerator::insetPolygon(
    const common::Polygon2D& polygon, double offset) {
  if (offset <= kEpsilon) return polygon;

  const int n = static_cast<int>(polygon.size());
  common::Polygon2D result;
  result.reserve(n);

  for (int i = 0; i < n; ++i) {
    const common::Node2D& a = polygon[(i - 1 + n) % n];
    const common::Node2D& b = polygon[i];
    const common::Node2D& c = polygon[(i + 1) % n];

    common::Node2D e1 = vecSub(b, a);
    common::Node2D e2 = vecSub(c, b);

    double len1 = std::hypot(e1.x, e1.y);
    double len2 = std::hypot(e2.x, e2.y);
    if (len1 < kEpsilon || len2 < kEpsilon) {
      result.push_back(b);
      continue;
    }

    // outward left normals (CW polygon: interior is right, outward is left)
    common::Node2D n1(-e1.y / len1, e1.x / len1);
    common::Node2D n2(-e2.y / len2, e2.x / len2);

    // offset edge 1: p = a + n1*offset + t * e1
    // offset edge 2: p = b + n2*offset + s * e2
    common::Node2D o1 = vecAdd(a, vecMul(n1, offset));
    common::Node2D o2 = vecAdd(b, vecMul(n2, offset));
    common::Node2D rhs = vecSub(o2, o1);

    // solve t*e1 - s*e2 = rhs
    // det = e1.x*(-e2.y) - (-e2.x)*e1.y = -cross2D(e1, e2)
    double det = -cross2D(e1, e2);

    if (std::fabs(det) < kEpsilon) {
      // parallel edges: use average normal
      result.push_back(
          common::Node2D(b.x + (n1.x + n2.x) * 0.5 * offset,
                         b.y + (n1.y + n2.y) * 0.5 * offset));
    } else {
      // t = cross(e2, rhs) / det
      double t = cross2D(e2, rhs) / det;
      common::Node2D isect = vecAdd(o1, vecMul(e1, t));

      // miter limit: cap offset distance at 4x offset to avoid self-intersection
      double dx = isect.x - b.x;
      double dy = isect.y - b.y;
      double dist = std::hypot(dx, dy);
      double max_miter = offset * 4.0;
      if (dist > max_miter) {
        double scale = max_miter / dist;
        isect.x = b.x + dx * scale;
        isect.y = b.y + dy * scale;
      }
      result.push_back(isect);
    }
  }

  return result;
}

void BoundaryPathGenerator::appendCornerArc(
    std::vector<common::Pose2D>& path, const common::Node2D& prev,
    const common::Node2D& corner, const common::Node2D& next,
    const BoundaryPathOptions& options) {
  double r = options.arc_radius;
  if (r <= kEpsilon) {
    double theta = std::atan2(next.y - corner.y, next.x - corner.x);
    path.emplace_back(corner.x, corner.y, theta);
    return;
  }

  common::Node2D e1 = vecSub(corner, prev);
  common::Node2D e2 = vecSub(next, corner);
  double l1 = std::hypot(e1.x, e1.y);
  double l2 = std::hypot(e2.x, e2.y);
  if (l1 < kEpsilon || l2 < kEpsilon) {
    return;
  }
  e1 = common::Node2D(e1.x / l1, e1.y / l1);
  e2 = common::Node2D(e2.x / l2, e2.y / l2);

  double cos_a = e1.x * e2.x + e1.y * e2.y;
  cos_a = std::clamp(cos_a, -1.0, 1.0);
  double alpha = std::acos(cos_a);

  // skip smoothing for nearly straight corners
  if (std::fabs(std::sin(alpha)) < 1e-4) {
    double theta = std::atan2(e2.y, e2.x);
    path.emplace_back(corner.x, corner.y, theta);
    return;
  }

  // desired tangent length along each edge
  double desired_tan = r / std::tan(alpha * 0.5);
  double max_tan = std::min(l1 * 0.45, l2 * 0.45);
  double tan_len = std::min(desired_tan, max_tan);

  if (tan_len < kEpsilon) {
    double theta = std::atan2(e2.y, e2.x);
    path.emplace_back(corner.x, corner.y, theta);
    return;
  }

  double actual_r = tan_len * std::tan(alpha * 0.5);

  common::Node2D entry(corner.x - e1.x * tan_len, corner.y - e1.y * tan_len);
  common::Node2D exit_pt(corner.x + e2.x * tan_len, corner.y + e2.y * tan_len);

  double cr = cross2D(e1, e2);
  // normal of e1 pointing toward arc center
  double n1x, n1y;
  if (cr > 0.0) {
    n1x = -e1.y;
    n1y = e1.x;  // left
  } else {
    n1x = e1.y;
    n1y = -e1.x;  // right
  }

  common::Node2D center(entry.x + n1x * actual_r, entry.y + n1y * actual_r);

  double theta_entry = std::atan2(e1.y, e1.x);
  path.emplace_back(entry.x, entry.y, theta_entry);

  double start_angle = std::atan2(entry.y - center.y, entry.x - center.x);
  double end_angle = std::atan2(exit_pt.y - center.y, exit_pt.x - center.x);
  double sweep = end_angle - start_angle;
  if (cr > 0.0 && sweep < 0.0) sweep += 2.0 * M_PI;
  if (cr < 0.0 && sweep > 0.0) sweep -= 2.0 * M_PI;

  for (int k = 1; k < options.arc_samples; ++k) {
    double t = static_cast<double>(k) / options.arc_samples;
    double angle = start_angle + sweep * t;
    double px = center.x + actual_r * std::cos(angle);
    double py = center.y + actual_r * std::sin(angle);
    double tangent = angle + M_PI_2;
    if (cr < 0.0) tangent = angle - M_PI_2;
    path.emplace_back(px, py, tangent);
  }

  double theta_exit = std::atan2(e2.y, e2.x);
  path.emplace_back(exit_pt.x, exit_pt.y, theta_exit);
}

std::vector<common::Pose2D>
BoundaryPathGenerator::traceSingleObstacleBoundary(
    const common::Polygon2D& obstacle,
    const BoundaryPathOptions& options) {
  // 1. cleanup
  common::Polygon2D clean = obstacle;
  if (!cleanupPolygon(&clean)) return {};

  // 2. normalize to desired orientation
  bool poly_cw = isClockwise(clean);
  if (poly_cw != options.clockwise) {
    std::reverse(clean.begin(), clean.end());
  }

  // 3. apply inset (outward offset for CW polygon)
  common::Polygon2D offset_poly = insetPolygon(clean, options.offset);

  // 4. cleanup again after offset
  if (!cleanupPolygon(&offset_poly)) return {};

  const int n = static_cast<int>(offset_poly.size());
  if (n < 3) return {};

  std::vector<common::Pose2D> path;

  if (options.arc_radius <= kEpsilon) {
    // simple vertex traversal, no arc smoothing
    for (int i = 0; i < n; ++i) {
      const common::Node2D& curr = offset_poly[i];
      const common::Node2D& next =
          i == 0 && path.empty() ? offset_poly[1] : offset_poly[(i + 1) % n];
      // for closing: use direction from first to last? Let's use next edge
      double theta = std::atan2(next.y - curr.y, next.x - curr.x);
      path.emplace_back(curr.x, curr.y, theta);
    }
    // close the loop
    if (!path.empty()) {
      const common::Node2D& first = offset_poly[0];
      const common::Node2D& last = offset_poly[n - 1];
      double theta = std::atan2(first.y - last.y, first.x - last.x);
      path.emplace_back(first.x, first.y, theta);
    }
  } else {
    // arc smoothing: traverse each vertex with corner arc
    for (int i = 0; i < n; ++i) {
      const common::Node2D& prev = offset_poly[(i - 1 + n) % n];
      const common::Node2D& curr = offset_poly[i];
      const common::Node2D& next = offset_poly[(i + 1) % n];
      appendCornerArc(path, prev, curr, next, options);
    }
    // close the loop with the closing arc
    if (!path.empty()) {
      const common::Node2D& prev = offset_poly[n - 1];
      const common::Node2D& curr = offset_poly[0];
      const common::Node2D& next = offset_poly[1 % n];
      appendCornerArc(path, prev, curr, next, options);
    }
  }

  return path;
}

std::vector<std::vector<common::Pose2D>>
BoundaryPathGenerator::generateObstacleBoundaries(
    const common::Polygon2D& /*boundary*/,
    const std::vector<common::Polygon2D>& obstacles,
    const BoundaryPathOptions& options) {
  std::vector<std::vector<common::Pose2D>> result;
  result.reserve(obstacles.size());
  for (const auto& obs : obstacles) {
    if (obs.size() < 3) continue;
    auto path = traceSingleObstacleBoundary(obs, options);
    if (!path.empty()) {
      result.push_back(std::move(path));
    }
  }
  return result;
}

nav_msgs::msg::Path BoundaryPathGenerator::generateGlobalBoundaryPath(
    const common::Polygon2D& boundary,
    const std::vector<common::Polygon2D>& obstacles,
    const BoundaryPathOptions& options,
    const geometry_msgs::msg::Pose& start_pose) {
  nav_msgs::msg::Path result;

  auto paths = generateObstacleBoundaries(boundary, obstacles, options);
  if (paths.empty()) return result;

  const size_t m = paths.size();

  // only one obstacle: just convert and return
  if (m == 1) {
    for (const auto& wp : paths[0]) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = wp.x;
      p.pose.position.y = wp.y;
      p.pose.position.z = 0.0;
      p.pose.orientation.z = std::sin(wp.theta / 2.0);
      p.pose.orientation.w = std::cos(wp.theta / 2.0);
      result.poses.push_back(p);
    }
    return result;
  }

  // for each obstacle index, store two orientations [0]=forward, [1]=reversed
  std::vector<std::vector<std::vector<common::Pose2D>>> oriented_paths(m);
  for (size_t i = 0; i < m; ++i) {
    oriented_paths[i].resize(2);
    oriented_paths[i][0] = paths[i];
    // reversed version
    auto reversed = paths[i];
    std::reverse(reversed.begin(), reversed.end());
    for (auto& wp : reversed) {
      wp.theta += M_PI;
      if (wp.theta > M_PI) wp.theta -= 2.0 * M_PI;
      if (wp.theta <= -M_PI) wp.theta += 2.0 * M_PI;
    }
    oriented_paths[i][1] = std::move(reversed);
  }

  // nearest-neighbor ordering starting from start_pose
  common::Node2D current(start_pose.position.x, start_pose.position.y);

  std::vector<bool> visited(m, false);
  std::vector<size_t> order;
  std::vector<int> orientations;

  for (size_t step = 0; step < m; ++step) {
    double best_dist = 1e18;
    int best_idx = -1;
    int best_ori = 0;

    for (size_t i = 0; i < m; ++i) {
      if (visited[i]) continue;
      for (int ori = 0; ori < 2; ++ori) {
        const auto& path = oriented_paths[i][ori];
        if (path.empty()) continue;
        double d = common::distance(current, path.front());
        if (d < best_dist) {
          best_dist = d;
          best_idx = static_cast<int>(i);
          best_ori = ori;
        }
      }
    }

    if (best_idx < 0) break;

    visited[best_idx] = true;
    order.push_back(static_cast<size_t>(best_idx));
    orientations.push_back(best_ori);
    current = oriented_paths[best_idx][best_ori].back();
  }

  // concatenate paths
  for (size_t k = 0; k < order.size(); ++k) {
    const auto& path = oriented_paths[order[k]][orientations[k]];
    if (path.empty()) continue;

    // add connecting segment from previous end to this path start
    if (k > 0) {
      const common::Pose2D& prev_end =
          oriented_paths[order[k - 1]][orientations[k - 1]].back();
      const common::Pose2D& curr_start = path.front();
      double dx = curr_start.x - prev_end.x;
      double dy = curr_start.y - prev_end.y;
      double dist = std::hypot(dx, dy);
      if (dist > 1e-4) {
        geometry_msgs::msg::PoseStamped p;
        p.pose.position.x = curr_start.x;
        p.pose.position.y = curr_start.y;
        p.pose.position.z = 0.0;
        double theta = std::atan2(dy, dx);
        p.pose.orientation.z = std::sin(theta / 2.0);
        p.pose.orientation.w = std::cos(theta / 2.0);
        result.poses.push_back(p);
      }
    }

    for (const auto& wp : path) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = wp.x;
      p.pose.position.y = wp.y;
      p.pose.position.z = 0.0;
      p.pose.orientation.z = std::sin(wp.theta / 2.0);
      p.pose.orientation.w = std::cos(wp.theta / 2.0);
      result.poses.push_back(p);
    }
  }

  return result;
}

}  // namespace complete_cover_path
}  // namespace global_planner
