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

  virtual void setStartPose(const geometry_msgs::msg::Pose& start_pose) {
    start_pose_ = start_pose;
  }

  virtual void setGoalPose(const geometry_msgs::msg::Pose& goal_pose) {
    goal_pose_ = goal_pose;
  }

  virtual nav_msgs::msg::Path searchPath() = 0;

 protected:
  std::shared_ptr<map::StaticMap> map_ = nullptr;
  const rclcpp::Logger& logger_;

  geometry_msgs::msg::Pose start_pose_;
  geometry_msgs::msg::Pose goal_pose_;
};
}  // namespace global_planner