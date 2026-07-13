#include "planner/global_planner/complete_cover_path/bcd.h"

#include <algorithm>
#include <iostream>
#include <list>
#include <set>
#include <stack>
#include <vector>

namespace global_planner {

namespace complete_cover_path {

namespace {
constexpr double epsilon = 1e-8;
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
                return a.pt.y > b.pt.y;
              }
              return a.pt.x > b.pt.y;
            });
  return sorted_points;
}

std::vector<Polygon2D> BCDDecomposer::decompose(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles) {
  // interval_id_counter_ = 0;

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

    if (cross(e_upper.p0, e_upper.p1, p_on_lower) > epsilon) {
      std::swap(e_lower, e_upper);
    }

    bool open_one =
        outOfBoundary(bound, obstacles, Node2D(node.pt.x + 1e-6, node.pt.y));

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
  if (isPointInPolygon(pt, bound)) {
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

}  // namespace complete_cover_path

}  // namespace global_planner