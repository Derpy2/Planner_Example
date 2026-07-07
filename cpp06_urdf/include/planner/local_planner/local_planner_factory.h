#pragma once

#include <memory>

#include "dwa.h"
#include "local_planner_base.h"
#include "mpc.h"
#include "pure_pursuit.h"

namespace local_planner {
enum LocalPlannerType { MPC = 0, DWA = 1, PURE_PURSUIT = 2 };

class LocalPlannerFactory {
 public:
  static std::unique_ptr<LocalPlannerBase> CreateLocalPlanner(
      const LocalPlannerType& type, std::shared_ptr<map::StaticMap> map,
      const rclcpp::Logger& logger) {
    switch (type) {
      case MPC: {
        return std::make_unique<LocalPlannerMPC>(map, logger);
      }
      case DWA: {
        return std::make_unique<LocalPlannerDWA>(map, logger);
      }
      case PURE_PURSUIT: {
        return std::make_unique<PurePursuit>(map, logger);
      }
      default:
        throw std::invalid_argument("Unknown local planner type");
    }
  }
};

}  // namespace local_planner