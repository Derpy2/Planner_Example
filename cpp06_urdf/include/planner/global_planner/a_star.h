#pragma once

#include <nav_msgs/msg/path.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>

#include "global_planner_base.h"

namespace global_planner {

class AStar : public GlobalPlannerBase {
 public:
  AStar(std::shared_ptr<map::StaticMap> map, const rclcpp::Logger& logger)
      : GlobalPlannerBase(map, logger) {}

  nav_msgs::msg::Path searchPath() override {
    return searchPath(start_pose_.position.x, start_pose_.position.y,
                      goal_pose_.position.x, goal_pose_.position.y);
  }

  nav_msgs::msg::Path searchPath(const double sx, const double sy,
                                 const double gx, const double gy);

 private:
  struct AStarNode {
    int x, y;
    double f;
    bool operator>(const AStarNode& o) const { return f > o.f; }
  };

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double costs[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
};
}  // namespace global_planner