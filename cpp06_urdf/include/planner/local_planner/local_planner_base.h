#pragma once

#include <rclcpp/rclcpp.hpp>

namespace local_planner {

class LocalPlannerBase {
 public:
  LocalPlannerBase() {}

  virtual ~LocalPlannerBase() = default;

 private:
};

}  // namespace local_planner