#pragma once

#include "local_planner_base.h"

namespace local_planner {

class MPC : public LocalPlannerBase {
 public:
  MPC() {}

  geometry_msgs::msg::Twist getControlCmd() override {
    geometry_msgs::msg::Twist cmd;
    return cmd;
  }

 private:
};

}  // namespace local_planner