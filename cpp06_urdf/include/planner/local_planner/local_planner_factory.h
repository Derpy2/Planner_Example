#pragma once

#include <memory>

#include "local_planner_base.h"
#include "lqr.h"
#include "mpc.h"

namespace local_planner {
enum LocalPlannerType { MPC = 0, LQR = 1, DWA = 2 };

class LocalPlannerFactory {
 public:
  static std::unique_ptr<LocalPlannerBase> CreateLocalPlanner(
      const LocalPlannerType& type) {
    switch (type) {
      case MPC: {
        return std::make_unique<MPC>();
      }
        //   case LQR: {
        //     return std::make_unique<LQR>();
        //   }
      default:
        throw std::invalid_argument("Unknown local planner type");
    }
  }
}

}  // namespace local_planner