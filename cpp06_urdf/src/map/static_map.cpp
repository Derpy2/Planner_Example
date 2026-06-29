#include "../include/map/static_map.h"

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
  addObstacle(-2.0, -1.0, 1.0, 2.0);
  addObstacle(1.0, -3.0, 2.0, 0.0);
  addObstacle(-3.5, 1.5, -1.5, 2.5);
  addObstacle(2.5, 1.0, 3.5, 3.5);
  // Walls around the map
  for (int x = 0; x < (int)width_; ++x) {
    map_msg_.data[idx(x, 0)] = 100;
    map_msg_.data[idx(x, height_ - 1)] = 100;
  }
  for (int y = 0; y < (int)height_; ++y) {
    map_msg_.data[idx(0, y)] = 100;
    map_msg_.data[idx(width_ - 1, y)] = 100;
  }
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
}

void StaticMap::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

void StaticMap::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

}  // namespace map