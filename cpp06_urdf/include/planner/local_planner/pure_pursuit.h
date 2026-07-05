#pragma once

#include "common/config.h"
#include "common/util.h"
#include "local_planner_base.h"

namespace local_planner {
using namespace common;

class PurePursuit : public LocalPlannerBase {
 public:
  PurePursuit(const rclcpp::Logger& logger) : LocalPlannerBase(logger) {}

  void Init() override {}

  geometry_msgs::msg::Twist getControlCmd() override;

 private:
};

}  // namespace local_planner