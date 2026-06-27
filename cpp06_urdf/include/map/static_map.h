#pragma once

#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace map {

class StaticMap {
 public:
  StaticMap();

  void buildMap();

  void addObstacle(double xmin, double ymin, double xmax, double ymax);

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

 private:
  double resolution_;
  unsigned int width_, height_;
  double origin_x_, origin_y_;
  nav_msgs::msg::OccupancyGrid map_msg_;
};
}  // namespace map