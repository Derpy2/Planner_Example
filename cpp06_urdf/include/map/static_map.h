#pragma once

#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/node2d.h"
#include "common/node3d.h"
#include "map/common/kd_tree.h"

namespace map {

using namespace common;

class StaticMap {
 public:
  StaticMap();

  void buildMap();

  void addObstacle(double xmin, double ymin, double xmax, double ymax);

  void addBoundary(double xmin, double ymin, double xmax, double ymax);

  void addObstacle(const Polygon2D& polygon);

  inline int idx(int x, int y) const { return y * (int)width_ + x; }

  inline bool inBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < (int)width_ && y < (int)height_;
  }

  void worldToGrid(double wx, double wy, int& gx, int& gy) const;

  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  const nav_msgs::msg::OccupancyGrid& getMapMsg() const { return map_msg_; }

  unsigned int width() const { return width_; }

  unsigned int height() const { return height_; }

  double resolution() const { return resolution_; }

  double origin_x() const { return origin_x_; }

  double origin_y() const { return origin_y_; }

  Polygon2D boundary() const { return boundary_; }

  std::vector<Polygon2D> obstacles() const { return obstacles_; }

  Polygon2D getBoundaryWorld();

  std::vector<Polygon2D> getObstaclesWorld();

  bool hasObstacleInRange(const std::vector<Node2D>& box);

  bool hasObstacleInRadius(const Node2D& point, const double radius);

 private:
  void buildKDTree();

 private:
  double resolution_;
  unsigned int width_, height_;
  double origin_x_, origin_y_;
  nav_msgs::msg::OccupancyGrid map_msg_;

  // 逆时针Polygon
  Polygon2D boundary_;
  std::vector<Polygon2D> obstacles_;
  std::shared_ptr<KDTree> kd_tree_;
};
}  // namespace map