#pragma once

#include "common/config.h"
#include "common/util.h"
#include "local_planner_base.h"

namespace local_planner {
using namespace common;

class PurePursuit : public LocalPlannerBase {
 public:
  PurePursuit(std::shared_ptr<map::StaticMap> map, const rclcpp::Logger logger)
      : LocalPlannerBase(map, logger) {}

  void init() override {}

  geometry_msgs::msg::Twist getControlCmd() override;

 private:
};

}  // namespace local_planner