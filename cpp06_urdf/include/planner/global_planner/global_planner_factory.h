#pragma once

#include <iostream>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "a_star.h"
#include "hybrid_a_star.h"

namespace global_planner {

enum GlobalPlannerType { A_STAR = 1, HYBRID_A_STAR = 2 };

class GlobalPlannerFactory {
 public:
  static std::unique_ptr<GlobalPlannerBase> CreateGlobalPlanner(
      const GlobalPlannerType& type, const std::shared_ptr<map::StaticMap>& map,
      const rclcpp::Logger& logger) {
    switch (type) {
      case A_STAR:
        return std::make_unique<AStar>(map, logger);
        //   case HYBRID_A_STAR:
        //     return std::make_unique<GlobalPlannerHybridAStar>();
      default:
        throw std::invalid_argument("Unknown globla planner type");
    }
  }
};
}  // namespace global_planner