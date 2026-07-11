#pragma once

#include <map>

#include "common/bcd_data.h"
#include "common/node2d.h"

namespace global_planner {

namespace complete_cover_path {

using namespace common;

class BCDDecomposer {
 public:
  BCDDecomposer() {}

  // 输入边缘，障碍物列表
  std::vector<Cell> decompose(const Polygon2D& boundary,
                              const std::vector<Polygon2D>& obstacles);

  // 单个Cell生成蛇形牛耕路径
  std::vector<Node2D> generateSnakePath(const Cell& cell, double cut_width);

  std::vector<EventPoint> generateEvents(const Polygon2D& bound,
                                         const std::vector<Polygon2D>& obstacle);

  EventType classifyVertex(const Node2D& v, const Polygon2D& polygon);

 private:
  std::vector<Segment2D> collectAllEdges(
      const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles);

  std::vector<YInterval> getYIntervals(double x_scan,
                                       const std::vector<ActiveEdge>& ael);

  std::map<int, std::vector<int>> matchIntervals(
      const std::vector<YInterval>& prev_inter,
      const std::vector<YInterval>& curr_inter);

  Cell buildCell(double x_l, double x_r, const YInterval& left_int,
                 const YInterval& right_int);

 private:
  int interval_id_counter_ = 0;
};

}  // namespace complete_cover_path

}  // namespace global_planner