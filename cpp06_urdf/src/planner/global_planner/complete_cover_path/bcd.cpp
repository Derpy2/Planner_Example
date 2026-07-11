#include "planner/global_planner/complete_cover_path/bcd.h"

#include <algorithm>
#include <set>

namespace global_planner {

namespace complete_cover_path {

namespace {
constexpr double epsilon = 1e-8;
}

std::vector<Cell> BCDDecomposer::decompose(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles) {
  std::vector<Cell> all_cells;
  interval_id_counter_ = 0;

  // 初始化事件和边
  std::vector<EventPoint> events = generateEvents(boundary, obstacles);
  std::vector<Segment2D> all_segs = collectAllEdges(boundary, obstacles);
  if (events.empty()) {
    return all_cells;
  }

  // AEL 活动边
  std::vector<ActiveEdge> ael;
  std::vector<YInterval> prev_intervals;
  double x_prev = events[0].pt.x;

  // 逐事件点推扫描线
  for (size_t e_idx = 0; e_idx < events.size(); ++e_idx) {
    const EventPoint& curr_ev = events[e_idx];
    double x_curr = curr_ev.pt.x;

    if (std::fabs(x_curr - x_prev) < epsilon) {
      continue;
    }

    std::vector<YInterval> curr_intervals = getYIntervals(x_curr, ael);

    std::map<int, std::vector<int>> match_map =
        matchIntervals(prev_intervals, curr_intervals);

    for (const YInterval& pi : prev_intervals) {
      if (!match_map.count(pi.id)) {
        continue;
      }
      const std::vector<int>& match_ids = match_map[pi.id];
      for (const int& cid : match_ids) {
        YInterval ci(0, 0, -1);
        for (const auto& tmp : curr_intervals) {
          if (tmp.id == cid) {
            ci = tmp;
            break;
          }
        }
        Cell new_cell = buildCell(x_prev, x_curr, pi, ci);
        all_cells.push_back(new_cell);
      }
    }

    // 更新AEL
    if (curr_ev.type == SPLIT || curr_ev.type == MERGE) {
      for (const auto& seg : curr_ev.adj_segs) {
        double y;
        if (segmentVerticalIntersect(seg, x_curr, y)) {
          ael.emplace_back(seg, x_curr, y);
        }
      }
    }

    // 滑动窗口更新
    prev_intervals.swap(curr_intervals);
    x_prev = x_curr;
  }

  return all_cells;
}

std::vector<Node2D> BCDDecomposer::generateSnakePath(const Cell& cell,
                                                     double cut_width) {
  std::vector<Node2D> path;
  double y_min = 1e18, y_max = -1e18;
  for (const auto& p : cell.polygon) {
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }

  bool flip = false;
  double y = y_min;
  while (y < y_max + epsilon) {
    if (!flip) {
      path.emplace_back(cell.x_left, y);
      path.emplace_back(cell.x_right, y);
    } else {
      path.emplace_back(cell.x_right, y);
      path.emplace_back(cell.x_left, y);
    }
    y += cut_width;
    flip = !flip;
  }
  return path;
}

std::vector<Segment2D> BCDDecomposer::collectAllEdges(
    const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles) {
  std::vector<Segment2D> edges;
  auto addPolySeg = [&](const Polygon2D& p) {
    int sz = p.size();

    for (int i = 0; i < sz; ++i) {
      int j = (i + 1) % sz;
      edges.emplace_back(p[i], p[j]);
    }
  };

  addPolySeg(boundary);
  for (auto& obs : obstacles) {
    addPolySeg(obs);
  }

  return edges;
}

EventType BCDDecomposer::classifyVertex(const Node2D& v,
                                        const Polygon2D& polygon) {
  int n = polygon.size();
  int idx = -1;
  for (int i = 0; i < n; ++i) {
    if (polygon[i] == v) {
      idx = i;
      break;
    }
  }

  if (idx == -1) {
    return NONE;
  }

  const Node2D& prev = polygon[(idx - 1 + n) % n];
  const Node2D& next = polygon[(idx + 1) % n];
  double cr = cross(prev, v, next);

  Node2D vin(prev.x - v.x, prev.y - v.y);
  Node2D vout(next.x - v.x, next.y - v.y);

  bool left_up = vin.x < epsilon;
  bool right_up = vout.x < epsilon;

  // SPLIT
  if (cr > epsilon && left_up && right_up) {
    return SPLIT;
  }
  // MERGE
  if (cr < -epsilon && !left_up && !right_up) {
    return MERGE;
  }

  if (vin.y > v.y + epsilon) {
    return NORMAL_UP;
  }
  return NORMAL_DOWN;
}

std::vector<EventPoint> BCDDecomposer::generateEvents(
    const Polygon2D& bound, const std::vector<Polygon2D>& obstacle) {
  std::set<EventPoint> ev_set;
  auto addPolyEvent = [&](const Polygon2D& p) {
    int n = p.size();
    for (int i = 0; i < n; ++i) {
      Node2D v = p[i];
      EventType t = classifyVertex(v, p);
      EventPoint e(v, t);
      e.adj_segs.emplace_back(p[(i - 1 + n) % n], v);
      e.adj_segs.emplace_back(v, p[(i + 1) % n]);
      ev_set.insert(e);
    }
  };

  addPolyEvent(bound);
  for (const Polygon2D& pt : obstacle) {
    addPolyEvent(pt);
  }

  return std::vector<EventPoint>(ev_set.begin(), ev_set.end());
}

std::vector<YInterval> BCDDecomposer::getYIntervals(
    double x_scan, const std::vector<ActiveEdge>& ael) {
  std::vector<double> ys;
  for (const ActiveEdge& edge : ael) {
    double y;
    if (segmentVerticalIntersect(edge.seg, x_scan, y)) {
      ys.push_back(y);
    }
  }
  std::sort(ys.begin(), ys.end());

  std::vector<YInterval> intervals;
  for (size_t i = 0; i + 1 < ys.size(); i += 2) {
    intervals.emplace_back(ys[i], ys[i + 1], interval_id_counter_++);
  }
  return intervals;
}

std::map<int, std::vector<int>> BCDDecomposer::matchIntervals(
    const std::vector<YInterval>& prev_inter,
    const std::vector<YInterval>& curr_inter) {
  std::map<int, std::vector<int>> link;
  for (auto& pi : prev_inter) {
    for (auto& ci : curr_inter) {
      double overlap_low = std::max(pi.y_low, ci.y_low);
      double overlap_high = std::min(pi.y_high, ci.y_high);
      if (overlap_high - overlap_low > epsilon) {
        link[pi.id].push_back(ci.id);
      }
    }
  }
  return link;
}

Cell BCDDecomposer::buildCell(double x_l, double x_r, const YInterval& left_int,
                              const YInterval& right_int) {
  Cell cell;
  cell.x_left = x_l;
  cell.x_right = x_r;
  cell.interval_ids.push_back(left_int.id);
  cell.interval_ids.push_back(right_int.id);

  cell.polygon = {Node2D(x_l, left_int.y_low), Node2D(x_r, right_int.y_low),
                  Node2D(x_r, right_int.y_high), Node2D(x_l, left_int.y_high)};
  return cell;
}

}  // namespace complete_cover_path

}  // namespace global_planner