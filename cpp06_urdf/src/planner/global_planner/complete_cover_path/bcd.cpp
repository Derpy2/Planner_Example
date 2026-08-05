#include "planner/global_planner/complete_cover_path/bcd.h"

#include <algorithm>
#include <iostream>
#include <list>
#include <set>
#include <stack>
#include <vector>

#include "planner/global_planner/a_star.h"
#include "planner/global_planner/complete_cover_path/sweep.h"

namespace global_planner {

namespace complete_cover_path {

namespace {
constexpr double epsilon = 1e-8;

// 将 nav_msgs::msg::Path 转换为 Pose2D 序列，航向角由相邻点方向计算
std::vector<Pose2D> pathToPose2D(const nav_msgs::msg::Path& path) {
  std::vector<Pose2D> result;
  result.reserve(path.poses.size());
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double x = path.poses[i].pose.position.x;
    double y = path.poses[i].pose.position.y;
    double theta = 0.0;
    if (i + 1 < path.poses.size()) {
      double dx = path.poses[i + 1].pose.position.x - x;
      double dy = path.poses[i + 1].pose.position.y - y;
      theta = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = x - path.poses[i - 1].pose.position.x;
      double dy = y - path.poses[i - 1].pose.position.y;
      theta = std::atan2(dy, dx);
    }
    result.emplace_back(x, y, theta);
  }
  return result;
}

// 将局部扫描路径反向，并翻转朝向角
std::vector<Pose2D> reversePath(const std::vector<Pose2D>& path) {
  std::vector<Pose2D> reversed;
  reversed.reserve(path.size());
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    double theta = it->theta + M_PI;
    if (theta > M_PI) {
      theta -= 2.0 * M_PI;
    } else if (theta <= -M_PI) {
      theta += 2.0 * M_PI;
    }
    reversed.emplace_back(it->x, it->y, theta);
  }
  return reversed;
}

}  // namespace

std::vector<Pose2D> BCDDecomposer::connectWithAStar(const Pose2D& start,
                                                    const Pose2D& goal) const {
  std::vector<Pose2D> connection;
  if (!map_) {
    return connection;
  }

  AStar planner(map_, logger_);
  nav_msgs::msg::Path path =
      planner.searchPath(start.x, start.y, goal.x, goal.y);
  if (path.poses.empty()) {
    RCLCPP_WARN(logger_,
                "A* failed to connect coverage cells, falling back to direct "
                "connection.");
    return connection;
  }

  connection = pathToPose2D(path);
  return connection;
}

Polygon2D BCDDecomposer::rotatePolygon(const Polygon2D& poly,
                                       double angle) const {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Polygon2D rotated;
  rotated.reserve(poly.size());
  for (const auto& pt : poly) {
    rotated.emplace_back(pt.x * c + pt.y * s, -pt.x * s + pt.y * c);
  }
  return rotated;
}

double BCDDecomposer::computeRotationAngle(const Node2D& dir) const {
  return std::atan2(dir.y, dir.x);
}

Polygon2D BCDDecomposer::preProcessPolygon(const Polygon2D& polygon) {
  // 忽略共线点
  int n = polygon.size();
  std::vector<int> erase_ids;
  for (int i = 0; i < n; ++i) {
    const Node2D& prev = polygon[(i - 1 + n) % n];
    const Node2D& next = polygon[(i + 1) % n];
    // 共线
    if (colinear(prev, polygon[i], next)) {
      erase_ids.emplace_back(i);
    }
  }

  size_t start_idx = 0;
  Polygon2D new_polygon;
  for (int i = 0; i < n; ++i) {
    if (erase_ids.size() > start_idx && erase_ids[start_idx] == i) {
      start_idx++;
      continue;
    }
    new_polygon.emplace_back(polygon[i]);
  }
  return new_polygon;
}

std::vector<BCDNode2D> BCDDecomposer::getSortedPoints(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles) {
  std::vector<BCDNode2D> sorted_points;
  int n = boundary.size();
  for (int i = 0; i < n; ++i) {
    sorted_points.emplace_back(BCDNode2D(boundary[i], true, i, -1));
  }

  int obs_size = obstacles.size();
  for (int i = 0; i < obs_size; ++i) {
    int n = obstacles[i].size();
    for (int j = 0; j < n; ++j) {
      sorted_points.emplace_back(BCDNode2D(obstacles[i][j], false, j, i));
    }
  }

  std::sort(sorted_points.begin(), sorted_points.end(),
            [](const BCDNode2D& a, const BCDNode2D& b) {
              if (std::fabs(a.pt.x - b.pt.x) < epsilon) {
                return a.pt.y < b.pt.y;
              }
              return a.pt.x < b.pt.x;
            });
  return sorted_points;
}

std::vector<Polygon2D> BCDDecomposer::decompose(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles) {
  Polygon2D new_boundary = preProcessPolygon(boundary);
  std::vector<Polygon2D> new_obstacles;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    new_obstacles.emplace_back(preProcessPolygon(obstacles[i]));
  }

  // 初始化事件和边
  std::vector<BCDNode2D> sorted_points =
      getSortedPoints(new_boundary, new_obstacles);

  std::vector<Node2D> processed_points;
  std::vector<Polygon2D> closed_polygons;
  std::list<Segment2D> L;
  std::list<Polygon2D> open_polygons;

  for (size_t i = 0; i < sorted_points.size(); ++i) {
    BCDNode2D& node = sorted_points[i];
    if (std::find(processed_points.begin(), processed_points.end(), node.pt) !=
        processed_points.end()) {
      continue;
    }
    processEvent(new_boundary, new_obstacles, node, &sorted_points,
                 &processed_points, &L, &open_polygons, &closed_polygons);
  }

  return closed_polygons;
}

std::vector<Polygon2D> BCDDecomposer::decompose(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles,
    const Node2D& decomp_dir) {
  double dir_len = std::hypot(decomp_dir.x, decomp_dir.y);
  if (dir_len < epsilon) {
    RCLCPP_ERROR(logger_,
                 "Decompose direction is zero vector, cannot decompose.");
    return {};
  }

  double angle = computeRotationAngle(decomp_dir);

  Polygon2D rotated_boundary = rotatePolygon(boundary, angle);
  std::vector<Polygon2D> rotated_obstacles;
  rotated_obstacles.reserve(obstacles.size());
  for (const auto& obs : obstacles) {
    rotated_obstacles.emplace_back(rotatePolygon(obs, angle));
  }

  std::vector<Polygon2D> rotated_cells =
      decompose(rotated_boundary, rotated_obstacles);

  std::vector<Polygon2D> cells;
  cells.reserve(rotated_cells.size());
  for (auto& cell : rotated_cells) {
    cells.emplace_back(preProcessPolygon(rotatePolygon(cell, -angle)));
  }

  return cells;
}

void BCDDecomposer::processEvent(const Polygon2D& bound,
                                 const std::vector<Polygon2D>& obstacles,
                                 BCDNode2D& node,
                                 std::vector<BCDNode2D>* sorted_points,
                                 std::vector<Node2D>* processed_points,
                                 std::list<Segment2D>* L,
                                 std::list<Polygon2D>* open_polygons,
                                 std::vector<Polygon2D>* closed_polygons) {
  // 经过点pt平行Y轴的直线与L的交点
  std::vector<Node2D> intersections = getIntersections(*L, node);

  Segment2D e_prev = Segment2D(node.pt, getPrev(bound, obstacles, node, 1).pt);
  Segment2D e_next = Segment2D(node.pt, getNext(bound, obstacles, node, 1).pt);

  // 忽略垂直边
  if (std::fabs(e_prev.p0.x - e_prev.p1.x) < epsilon) {
    e_prev = Segment2D(e_prev.p1, getPrev(bound, obstacles, node, 2).pt);
  } else if (std::fabs(e_next.p0.x - e_next.p1.x) < epsilon) {
    e_next = Segment2D(e_next.p1, getNext(bound, obstacles, node, 2).pt);
  }

  std::function<bool(Node2D, Node2D)> less_x = [&](const Node2D& a,
                                                   const Node2D b) -> bool {
    return a.x < b.x || (std::fabs(a.x - b.x) < epsilon && a.y < b.y);
  };
  std::function<bool(Node2D, Node2D)> less_y = [&](const Node2D& a,
                                                   const Node2D b) -> bool {
    return a.y < b.y || (std::fabs(a.y - b.y) < epsilon && a.x < b.x);
  };

  Segment2D e_lower = e_prev;
  Segment2D e_upper = e_next;
  if (less_x(e_prev.p1, e_prev.p0) && less_x(e_next.p1, e_next.p0)) {
    Node2D p_on_upper = e_lower.p0 == e_upper.p0 ? e_upper.p1 : e_upper.p0;
    // 两个向量都朝左
    if (cross(e_lower.p0, e_lower.p1, p_on_upper) > epsilon) {
      std::swap(e_lower, e_upper);
    }

    bool close_one =
        outOfBoundary(bound, obstacles, Node2D(node.pt.x + 1e-6, node.pt.y));

    // Fine edges to remove
    std::list<Segment2D>::iterator e_lower_it = L->begin();
    size_t e_lower_id = 0;
    for (; e_lower_it != L->end(); ++e_lower_it) {
      if (*e_lower_it == e_lower || *e_lower_it == e_lower.opposite()) {
        break;
      }
      e_lower_id++;
    }

    std::list<Segment2D>::iterator e_upper_it = std::next(e_lower_it);
    size_t e_upper_id = e_lower_id + 1;
    size_t lower_cell_id = e_lower_id / 2;
    size_t upper_cell_id = e_upper_id / 2;

    if (close_one) {
      std::list<Polygon2D>::iterator cell =
          std::next(open_polygons->begin(), lower_cell_id);
      cell->push_back(e_lower.p0);
      if (!(e_lower.p0 == e_upper.p0)) {
        cell->push_back(e_upper.p0);
      }

      if (cleanupPolygon(&*cell)) {
        closed_polygons->push_back(*cell);
      }
      L->erase(e_lower_it);
      L->erase(e_upper_it);
      open_polygons->erase(cell);
    } else {
      // close two cell and open one

      // close lower cell
      std::list<Polygon2D>::iterator lower_cell =
          std::next(open_polygons->begin(), lower_cell_id);
      lower_cell->push_back(intersections[e_lower_id - 1]);
      lower_cell->push_back(intersections[e_lower_id]);

      if (cleanupPolygon(&*lower_cell)) {
        closed_polygons->push_back(*lower_cell);
      }
      // close upper cell
      std::list<Polygon2D>::iterator upper_cell =
          std::next(open_polygons->begin(), upper_cell_id);
      upper_cell->push_back(intersections[e_upper_id]);
      upper_cell->push_back(intersections[e_upper_id + 1]);
      if (cleanupPolygon(&*upper_cell)) {
        closed_polygons->push_back(*upper_cell);
      }
      // delete cells from list
      L->erase(e_lower_it);
      L->erase(e_upper_it);
      // open new cell
      std::list<Polygon2D>::iterator new_polygon =
          open_polygons->insert(lower_cell, Polygon2D());
      new_polygon->push_back(intersections[e_upper_id + 1]);
      new_polygon->push_back(intersections[e_lower_id - 1]);
      open_polygons->erase(lower_cell);
      open_polygons->erase(upper_cell);
    }

    processed_points->push_back(e_lower.p0);
    if (!(e_lower.p0 == e_upper.p0)) {
      processed_points->push_back(e_upper.p0);
    }

  } else if (!less_x(e_prev.p1, e_prev.p0) && !less_x(e_next.p1, e_next.p0)) {
    // 两个向量都朝右
    Node2D p_on_lower = (e_lower.p0 == e_upper.p0) ? e_lower.p1 : e_lower.p0;
    // 确保沿Y轴方向lower在下方，upper在上方
    if (cross(e_upper.p0, e_upper.p1, p_on_lower) > epsilon) {
      std::swap(e_lower, e_upper);
    }

    bool open_one =
        outOfBoundary(bound, obstacles, Node2D(node.pt.x - 1e-6, node.pt.y));

    // Find edge to update
    size_t e_lower_id = 0;
    bool found_e_lower_id = false;

    for (int i = 0; i < static_cast<int>(intersections.size()) - 1; i += 2) {
      if (intersections.empty()) {
        break;
      }

      if (open_one) {
        if (less_y(intersections[i], e_lower.p0) &&
            less_y(intersections[i + 1], e_upper.p0)) {
          e_lower_id = i;
          found_e_lower_id = true;
        }
      } else {
        if (less_y(intersections[i], e_lower.p0) &&
            less_y(e_upper.p0, intersections[i + 1])) {
          e_lower_id = i;
        }
      }
    }

    if (open_one) {
      // Add one cell above e_upper
      std::list<Segment2D>::iterator e_upper_it = L->begin();
      std::list<Polygon2D>::iterator open_cell = open_polygons->begin();

      if (!L->empty() && found_e_lower_id) {
        e_upper_it = std::next(e_upper_it, e_lower_id + 1);
        open_cell = std::next(open_cell, e_lower_id / 2 + 1);
      }

      // update edge list
      if (L->empty()) {
        L->insert(L->end(), e_lower);
        L->insert(L->end(), e_upper);
      } else if (!L->empty() && !found_e_lower_id) {
        L->insert(L->begin(), e_upper);
        L->insert(L->begin(), e_lower);
      } else {
        std::list<Segment2D>::iterator inserter = std::next(e_upper_it);
        L->insert(inserter, e_lower);
        L->insert(inserter, e_upper);
      }

      // create new polygon
      std::list<Polygon2D>::iterator open_polygon =
          open_polygons->insert(open_cell, Polygon2D());
      open_polygon->push_back(e_upper.p0);
      if (!(e_lower.p0 == e_upper.p0)) {
        open_polygon->push_back(e_lower.p0);
      }
    } else {
      // Add new polygon between e_lower and e_upper
      std::list<Segment2D>::iterator e_lower_id_it =
          std::next(L->begin(), e_lower_id);
      std::list<Polygon2D>::iterator cell =
          std::next(open_polygons->begin(), e_lower_id / 2);

      // Add e_lower and e_upper
      std::list<Segment2D>::iterator e_lower_it =
          L->insert(std::next(e_lower_id_it), e_lower);
      L->insert(std::next(e_lower_it), e_upper);

      if (intersections.size() < e_lower_id + 2) {
        return;
      }

      // Add new cell
      std::list<Polygon2D>::iterator new_polygon =
          open_polygons->insert(cell, Polygon2D());

      // Close one cell
      cell->push_back(intersections[e_lower_id]);
      cell->push_back(intersections[e_lower_id + 1]);

      if (cleanupPolygon(&*cell)) {
        closed_polygons->push_back(*cell);
      }

      // Open two new cell
      new_polygon->push_back(e_lower.p0);
      new_polygon->push_back(intersections[e_lower_id]);

      new_polygon = open_polygons->insert(cell, Polygon2D());
      new_polygon->push_back(intersections[e_lower_id + 1]);
      new_polygon->push_back(e_upper.p0);

      // Close old cell
      open_polygons->erase(cell);
    }
    processed_points->push_back(e_lower.p0);
    if (!(e_lower.p0 == e_upper.p0)) {
      processed_points->push_back(e_upper.p0);
    }
  } else {
    BCDNode2D node_middle = node;
    std::list<Segment2D>::iterator it = L->end();
    while (it == L->end()) {
      for (it = L->begin(); it != L->end(); it++) {
        if (node_middle.pt == it->p0 || node_middle.pt == it->p1) {
          if (!(node.pt == node_middle.pt)) {
            std::vector<BCDNode2D>::iterator i_v = sorted_points->end();
            std::vector<BCDNode2D>::iterator i_v_middle = sorted_points->end();
            for (std::vector<BCDNode2D>::iterator it = sorted_points->begin();
                 it != sorted_points->end(); ++it) {
              if (it->pt == node.pt) {
                i_v = it;
              }
              if (it->pt == node_middle.pt) {
                i_v_middle = it;
              }
            }

            std::iter_swap(i_v, i_v_middle);
          }
          break;
        }
      }

      if (it == L->end()) {
        BCDNode2D node_prev = getPrev(bound, obstacles, node_middle, 1);
        BCDNode2D node_next = getNext(bound, obstacles, node_middle, 1);

        if (std::fabs(node_prev.pt.x - node_middle.pt.x) < epsilon) {
          node_middle = node_prev;
        } else {
          node_middle = node_next;
        }
      }
    }

    // 垂直边处理
    e_prev =
        Segment2D(node_middle.pt, getPrev(bound, obstacles, node_middle, 1).pt);
    e_next =
        Segment2D(node_middle.pt, getNext(bound, obstacles, node_middle, 1).pt);

    // Find edge to update
    std::list<Segment2D>::iterator old_e_it = L->begin();
    Segment2D new_edge;
    size_t edge_id = 0;
    for (; old_e_it != L->end(); ++old_e_it) {
      if (*old_e_it == e_next || *old_e_it == e_next.opposite()) {
        new_edge = e_prev;
        break;
      } else if (*old_e_it == e_prev || *old_e_it == e_prev.opposite()) {
        new_edge = e_next;
        break;
      }
      edge_id++;
    }

    // Update cell with new point
    size_t cell_id = edge_id / 2;
    std::list<Polygon2D>::iterator cell =
        std::next(open_polygons->begin(), cell_id);

    if ((edge_id % 2) == 0) {
      // insert point at end
      cell->push_back(new_edge.p0);
    } else {
      // insert point at begin
      cell->insert(cell->begin(), new_edge.p0);
    }

    // update edge
    L->insert(old_e_it, new_edge);
    L->erase(old_e_it);

    processed_points->push_back(node_middle.pt);
  }
}

bool BCDDecomposer::cleanupPolygon(Polygon2D* poly) {
  bool erased = true;
  while (erased) {
    erased = false;
    int n = poly->size();
    for (int i = 0; i < n; ++i) {
      const Node2D& prev = (*poly)[(i - 1 + n) % n];
      const Node2D& curr = (*poly)[i];
      const Node2D& next = (*poly)[(i + 1) % n];

      if (curr == prev || curr == next) {
        poly->erase(poly->begin() + i);
        erased = true;
        break;
      }
    }
  }

  if (poly->size() < 3) {
    return false;
  }

  int n = poly->size();
  for (int i = 0; i < n; ++i) {
    const Node2D& prev = (*poly)[(i - 1 + n) % n];
    const Node2D& curr = (*poly)[i];
    const Node2D& next = (*poly)[(i + 1) % n];
    if (colinear(prev, curr, next)) {
      poly->erase(poly->begin() + i);
      --i;
      --n;
    }
  }

  return poly->size() >= 3;
}

bool BCDDecomposer::outOfBoundary(const Polygon2D& bound,
                                  const std::vector<Polygon2D>& obstacles,
                                  const Node2D& pt) {
  // 在 boundary 外
  if (!isPointInPolygon(pt, bound)) {
    return true;
  }

  // 在 obstacles 内
  for (const Polygon2D& obs : obstacles) {
    if (isPointInPolygon(pt, obs)) {
      return true;
    }
  }

  return false;
}

BCDNode2D BCDDecomposer::getPrev(const Polygon2D& bound,
                                 const std::vector<Polygon2D>& obstacles,
                                 const BCDNode2D& node, int num) {
  if (node.is_boundary) {
    int n = bound.size();
    int idx = (node.polygon_idx - num + n) % n;
    return BCDNode2D(bound[idx], true, idx, -1);
  } else if (node.obs_idx != -1) {
    int n = obstacles[node.obs_idx].size();
    int idx = (node.polygon_idx - num + n) % n;

    return BCDNode2D(obstacles[node.obs_idx][idx], false, idx, node.obs_idx);
  }
  return BCDNode2D(Node2D(), false, -1, -1);
}

BCDNode2D BCDDecomposer::getNext(const Polygon2D& bound,
                                 const std::vector<Polygon2D>& obstacles,
                                 const BCDNode2D& node, int num) {
  if (node.is_boundary) {
    int n = bound.size();
    int idx = (node.polygon_idx + num) % n;
    return BCDNode2D(bound[idx], true, idx, -1);
  } else if (node.obs_idx != -1) {
    int n = obstacles[node.obs_idx].size();
    int idx = (node.polygon_idx + num) % n;
    return BCDNode2D(obstacles[node.obs_idx][idx], false, idx, node.obs_idx);
  }
  return BCDNode2D(Node2D(), false, -1, -1);
}

std::vector<Node2D> BCDDecomposer::getIntersections(
    const std::list<Segment2D>& L, const BCDNode2D& node) {
  std::vector<Node2D> intersections;
  const Node2D& pt = node.pt;
  for (const auto& seg : L) {
    // 判断是否与直线重合
    if (std::fabs(seg.p0.x - seg.p1.x) < epsilon &&
        std::fabs(pt.x - seg.p0.x) < epsilon) {
      intersections.emplace_back(seg.p1);
    } else {
      double y_cross;
      if (segmentVerticalIntersect(seg, pt.x, y_cross)) {
        Node2D intersect = Node2D(pt.x, y_cross);
        intersections.emplace_back(intersect);
      } else {
        std::cout << "No intersection found" << std::endl;
      }
    }
  }
  return intersections;
}

std::vector<Pose2D> BCDDecomposer::generateGlobalCoverPath(
    const std::vector<Polygon2D>& cells, double offset, const Node2D& dir) {
  std::vector<Pose2D> global_path;
  const int n = static_cast<int>(cells.size());
  if (n == 0) {
    return global_path;
  }

  // 为每个 cell 生成正向与反向的局部扫描路径
  std::vector<std::vector<Pose2D>> local_paths[2];
  // local_paths[0].resize(n);
  // local_paths[1].resize(n);

  for (int i = 0; i < n; ++i) {
    std::vector<Pose2D> local_path;
    // 跳过无法找到扫描线路径的cell
    if (!Sweep::computeSweep(cells[i], offset, dir, &local_path) ||
        local_path.empty()) {
      continue;
    }
    local_paths[0].emplace_back(local_path);
    local_paths[1].emplace_back(reversePath(local_path));
  }

  if (n == 1) {
    global_path = std::move(local_paths[0][0]);
    return global_path;
  }

  // TSP DP：状态 (mask, last, ori)，记录到达最后一个 cell 的最小转移距离
  // dp[mask][last][ori]
  const int vaild_n = local_paths[0].size();
  const int max_mask = 1 << vaild_n;
  const double INF = 1e18;
  std::vector<std::vector<std::array<double, 2>>> dp(
      max_mask, std::vector<std::array<double, 2>>(vaild_n, {INF, INF}));
  // parent[mask][last][ori] = {prev_last, prev_ori}
  std::vector<std::vector<std::array<std::pair<int, int>, 2>>> parent(
      max_mask, std::vector<std::array<std::pair<int, int>, 2>>(
                    vaild_n, {{{-1, -1}, {-1, -1}}}));

  for (int i = 0; i < vaild_n; ++i) {
    dp[1 << i][i][0] = 0.0;
    dp[1 << i][i][1] = 0.0;
  }

  for (int mask = 1; mask < max_mask; ++mask) {
    for (int last = 0; last < vaild_n; ++last) {
      if (!(mask & (1 << last))) {
        continue;
      }
      for (int ori = 0; ori < 2; ++ori) {
        if (dp[mask][last][ori] >= INF) {
          continue;
        }
        const Pose2D& exit_pt = local_paths[ori][last].back();
        for (int next = 0; next < vaild_n; ++next) {
          if (mask & (1 << next)) {
            continue;
          }
          for (int next_ori = 0; next_ori < 2; ++next_ori) {
            const Pose2D& entry_pt = local_paths[next_ori][next].front();
            double cost = distance(exit_pt, entry_pt);
            int new_mask = mask | (1 << next);
            if (dp[new_mask][next][next_ori] > dp[mask][last][ori] + cost) {
              dp[new_mask][next][next_ori] = dp[mask][last][ori] + cost;
              parent[new_mask][next][next_ori] = {last, ori};
            }
          }
        }
      }
    }
  }

  // 找出最优终点
  int full_mask = max_mask - 1;
  double best_cost = INF;
  int best_last = -1;
  int best_ori = -1;
  for (int last = 0; last < vaild_n; ++last) {
    for (int ori = 0; ori < 2; ++ori) {
      if (dp[full_mask][last][ori] < best_cost) {
        best_cost = dp[full_mask][last][ori];
        best_last = last;
        best_ori = ori;
      }
    }
  }

  if (best_last < 0) {
    return {};
  }

  // 回溯得到访问顺序
  std::vector<std::pair<int, int>> order;
  int mask = full_mask;
  int last = best_last;
  int ori = best_ori;
  while (true) {
    order.emplace_back(last, ori);
    auto& pr = parent[mask][last][ori];
    if (pr.first < 0) {
      break;
    }
    mask ^= (1 << last);
    last = pr.first;
    ori = pr.second;
  }
  std::reverse(order.begin(), order.end());

  // 按顺序拼接局部路径，若提供地图则在相邻局部路径之间用 A* 连接
  for (size_t i = 0; i < order.size(); ++i) {
    const auto& local = local_paths[order[i].second][order[i].first];

    if (i > 0 && map_) {
      const Pose2D& prev_exit = global_path.back();
      const Pose2D& next_entry = local.front();
      std::vector<Pose2D> connection = connectWithAStar(prev_exit, next_entry);

      if (!connection.empty()) {
        // A* 路径起点为 prev_exit，终点为 next_entry；跳过这两个端点以避免
        // 与前后局部路径重复。当路径只有一点时保留该点。
        size_t start_idx = (connection.size() > 1) ? 1 : 0;
        size_t end_idx = connection.size() - ((connection.size() > 1) ? 1 : 0);
        global_path.insert(global_path.end(), connection.begin() + start_idx,
                           connection.begin() + end_idx);
      }
    }

    global_path.insert(global_path.end(), local.begin(), local.end());
  }

  return global_path;
}

}  // namespace complete_cover_path

}  // namespace global_planner