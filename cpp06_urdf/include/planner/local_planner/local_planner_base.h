#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace local_planner {

class LocalPlannerBase {
 public:
  LocalPlannerBase() {}

  virtual ~LocalPlannerBase() = default;

  virtual geometry_msgs::msg::Twist getControlCmd() = 0;

  void setNearsetIdx(const size_t idx) { nearest_idx_ = idx; }

  void setSmoothedPath(const nav_msgs::msg::Path& smoothed_local) {
    smoothed_local_ = smoothed_local;
  }

  void setCurrentPose(const geometry_msgs::msg::PoseWithCovariance pose) {
    current_pose_ = pose;
  }

 protected:
  size_t nearest_idx_;
  nav_msgs::msg::Path smoothed_local_;
  geometry_msgs::msg::PoseWithCovariance current_pose_;
};

}  // namespace local_planner