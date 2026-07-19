#pragma once

#include <list>
#include <map>

#include "common/bcd_data.h"
#include "common/node2d.h"

namespace global_planner {

namespace complete_cover_path {

using namespace common;

class BCDDecomposer {
 public:
  BCDDecomposer() {}

  Polygon2D preProcessPolygon(const Polygon2D& polygon);

  // 输入边缘，障碍物列表
  std::vector<Polygon2D> decompose(const Polygon2D& boundary,
                                   const std::vector<Polygon2D>& obstacles);

  // 对每个 cell 生成局部扫描路径，并通过 TSP 优化 cell 访问顺序，
  // 返回总长度最短的全局覆盖路径
  std::vector<Pose2D> generateGlobalCoverPath(
      const std::vector<Polygon2D>& cells, double offset, const Node2D& dir);

 private:
  std::vector<BCDNode2D> getSortedPoints(
      const Polygon2D& boundary, const std::vector<Polygon2D>& obstacles);

  void processEvent(const Polygon2D& bound,
                    const std::vector<Polygon2D>& obstacles, BCDNode2D& node,
                    std::vector<BCDNode2D>* sorted_points,
                    std::vector<Node2D>* processed_points,
                    std::list<Segment2D>* L,
                    std::list<Polygon2D>* open_polygons,
                    std::vector<Polygon2D>* closed_polygons);

  bool cleanupPolygon(Polygon2D* poly);

  bool outOfBoundary(const Polygon2D& bound,
                     const std::vector<Polygon2D>& obstacles, const Node2D& pt);

  BCDNode2D getPrev(const Polygon2D& bound,
                    const std::vector<Polygon2D>& obstacles,
                    const BCDNode2D& node, int num);

  BCDNode2D getNext(const Polygon2D& bound,
                    const std::vector<Polygon2D>& obstacles,
                    const BCDNode2D& node, int num);

  std::vector<Node2D> getIntersections(const std::list<Segment2D>& L,
                                       const BCDNode2D& node);

 private:
  int interval_id_counter_ = 0;
};

}  // namespace complete_cover_path

}  // namespace global_planner