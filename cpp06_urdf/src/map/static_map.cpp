#include "map/static_map.h"

#include <algorithm>

namespace map {
StaticMap::StaticMap() {
  resolution_ = 0.05;  // 0.05 m per cell
  width_ = 200;        // 10 m
  height_ = 200;
  origin_x_ = -5.0;
  origin_y_ = -5.0;
  buildMap();
}

void StaticMap::buildMap() {
  obstacles_.clear();
  map_msg_.header.frame_id = "map";
  map_msg_.header.stamp = rclcpp::Clock().now();
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = width_;
  map_msg_.info.height = height_;
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(width_ * height_, 0);
  // Rectangular obstacles (xmin, ymin, xmax, ymax)
  addBoundary(-5.0, -5.0, 5.0, 5.0);
  // addObstacle(-2.0, -1.0, 1.0, 2.0);
  addObstacle(1.0, -3.0, 2.0, 0.0);
  addObstacle(-3.5, 1.5, -1.5, 2.5);
  addObstacle(2.5, 1.0, 3.5, 3.5);

  Polygon2D obs1 =
      Polygon2D{Node2D(0.0, 0.0), Node2D(1.0, 0.0), Node2D(1.0, 1.0)};
  addObstacle(obs1);

  // Walls around the map
  for (int x = 0; x < (int)width_; ++x) {
    map_msg_.data[idx(x, 0)] = 100;
    map_msg_.data[idx(x, height_ - 1)] = 100;
  }
  for (int y = 0; y < (int)height_; ++y) {
    map_msg_.data[idx(0, y)] = 100;
    map_msg_.data[idx(width_ - 1, y)] = 100;
  }
  buildKDTree();
}

void StaticMap::addBoundary(double xmin, double ymin, double xmax,
                            double ymax) {
  int gx0, gy0, gx1, gy1;
  worldToGrid(xmin, ymin, gx0, gy0);
  worldToGrid(xmax, ymax, gx1, gy1);

  boundary_.clear();
  boundary_.emplace_back(gx0, gy0);
  boundary_.emplace_back(gx1, gy0);
  boundary_.emplace_back(gx1, gy1);
  boundary_.emplace_back(gx0, gy1);
}

void StaticMap::addObstacle(double xmin, double ymin, double xmax,
                            double ymax) {
  int gx0, gy0, gx1, gy1;
  worldToGrid(xmin, ymin, gx0, gy0);
  worldToGrid(xmax, ymax, gx1, gy1);
  for (int y = gy0; y <= gy1; ++y) {
    for (int x = gx0; x <= gx1; ++x) {
      if (inBounds(x, y)) {
        map_msg_.data[idx(x, y)] = 100;
      }
    }
  }

  Polygon2D polygon;
  polygon.emplace_back(gx0, gy0);
  polygon.emplace_back(gx1, gy0);
  polygon.emplace_back(gx1, gy1);
  polygon.emplace_back(gx0, gy1);
  obstacles_.emplace_back(polygon);
}

void StaticMap::addObstacle(const Polygon2D& polygon) {
  if (polygon.size() < 3) {
    return;
  }
  double min_x = polygon[0].x, max_x = polygon[0].x;
  double min_y = polygon[0].y, max_y = polygon[0].y;
  for (const auto& pt : polygon) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }
  int gx_min, gy_min, gx_max, gy_max;
  worldToGrid(min_x, min_y, gx_min, gy_min);
  worldToGrid(max_x, max_y, gx_max, gy_max);
  gx_min = std::max(0, gx_min);
  gy_min = std::max(0, gy_min);
  gx_max = std::min((int)width_ - 1, gx_max);
  gy_max = std::min((int)height_ - 1, gy_max);

  for (int y = gy_min; y <= gy_max; ++y) {
    for (int x = gx_min; x <= gx_max; ++x) {
      double wx, wy;
      gridToWorld(x, y, wx, wy);
      Node2D pt(wx, wy);
      if (isPointInPolygon(pt, polygon)) {
        map_msg_.data[idx(x, y)] = 100;
      }
    }
  }

  Polygon2D grid_poly;
  int gx, gy;
  for (const auto& pt : polygon) {
    worldToGrid(pt.x, pt.y, gx, gy);
    grid_poly.emplace_back(Node2D(gx, gy));
  }

  obstacles_.emplace_back(grid_poly);
}

void StaticMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

void StaticMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

common::Polygon2D StaticMap::getBoundaryWorld() {
  common::Polygon2D bound_2d;
  double wx, wy;
  bound_2d.resize(boundary_.size());
  for (auto& pt : boundary_) {
    gridToWorld(pt.x, pt.y, wx, wy);
    bound_2d.emplace_back(Node2D(wx, wy));
  }
  return bound_2d;
}

std::vector<common::Polygon2D> StaticMap::getObstaclesWorld() {
  std::vector<common::Polygon2D> obstacles_2d;
  double wx, wy;
  for (auto& obs : obstacles_) {
    common::Polygon2D obstacle;
    for (auto& pt : obs) {
      gridToWorld(pt.x, pt.y, wx, wy);
      obstacle.emplace_back(wx, wy);
    }
    obstacles_2d.emplace_back(obstacle);
  }
  return obstacles_2d;
}

void StaticMap::buildKDTree() {
  if (kd_tree_ == nullptr) {
    kd_tree_ = std::make_shared<KDTree>();
  }

  std::vector<Node2D> obstacle_points;
  obstacle_points.reserve(width_ * height_ / 10);

  for (int y = 0; y < (int)height_; ++y) {
    for (int x = 0; x < (int)width_; ++x) {
      if (map_msg_.data[idx(x, y)] == 100) {
        double wx, wy;
        gridToWorld(x, y, wx, wy);
        obstacle_points.emplace_back(wx, wy);
      }
    }
  }

  kd_tree_->build(obstacle_points);
}

bool StaticMap::hasObstacleInRange(const std::vector<Node2D>& box) {
  std::vector<Node2D> result;
  BoxPointType kd_tree_box;
  kd_tree_box.vertex_min[0] = box[0].x;
  kd_tree_box.vertex_min[1] = box[0].y;
  kd_tree_box.vertex_max[0] = box[1].x;
  kd_tree_box.vertex_max[1] = box[1].y;
  kd_tree_->boxSearch(kd_tree_box, result);

  return !result.empty();
}

bool StaticMap::hasObstacleInRadius(const Node2D& point, const double radius) {
  std::vector<Node2D> result;
  kd_tree_->radiusSearch(point, radius, result);
  return !result.empty();
}

}  // namespace map
