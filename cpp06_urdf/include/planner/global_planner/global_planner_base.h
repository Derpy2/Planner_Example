#pragma once

#include <rclcpp/rclcpp.hpp>

#include "map/static_map.h"

namespace global_planner {
class GlobalPlannerBase {
 public:
  GlobalPlannerBase(std::shared_ptr<map::StaticMap> map,
                    const rclcpp::Logger& logger)
      : map_(map), logger_(logger) {}
  virtual ~GlobalPlannerBase() = default;
  virtual nav_msgs::msg::Path searchPath(const double sx, const double sy,
                                         const double gx, const double gy) = 0;

 protected:
  std::shared_ptr<map::StaticMap> map_ = nullptr;
  const rclcpp::Logger& logger_;
};
}  // namespace global_planner