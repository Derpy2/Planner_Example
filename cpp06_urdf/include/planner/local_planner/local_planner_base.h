#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "map/static_map.h"

namespace local_planner {

class LocalPlannerBase {
 public:
  LocalPlannerBase(std::shared_ptr<map::StaticMap> map,
                   const rclcpp::Logger logger)
      : map_(map), logger_(logger) {}

  virtual ~LocalPlannerBase() = default;

  virtual void init() {}

  virtual geometry_msgs::msg::Twist getControlCmd() = 0;

  void setNearsetIdx(const size_t idx) { nearest_idx_ = idx; }

  void setSmoothedPath(const nav_msgs::msg::Path& smoothed_local) {
    smoothed_local_ = smoothed_local;
  }

  void setCurrentPose(const geometry_msgs::msg::PoseWithCovariance& pose) {
    current_pose_ = pose;
  }

  void setCurrentTwist(const geometry_msgs::msg::TwistWithCovariance& twist) {
    current_twist_ = twist;
  }

  virtual void visualizeSampledTrajectories(
      const std::string& frame_id = "map") {}

  // 最近一次求解使用的参考轨迹（默认空，MPC 实现返回重建后的轨迹）
  virtual nav_msgs::msg::Path getReferenceTrajectory() const {
    return nav_msgs::msg::Path();
  }

  virtual visualization_msgs::msg::MarkerArray getTrajectoryMarkers(
      const std::string& frame_id = "map") {
    (void)frame_id;
    return visualization_msgs::msg::MarkerArray();
  }

 protected:
  std::shared_ptr<map::StaticMap> map_ = nullptr;
  size_t nearest_idx_;
  nav_msgs::msg::Path smoothed_local_;
  geometry_msgs::msg::PoseWithCovariance current_pose_;
  geometry_msgs::msg::TwistWithCovariance current_twist_;
  const rclcpp::Logger logger_;
};

}  // namespace local_planner