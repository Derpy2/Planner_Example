#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "planner/local_planner/local_planner_factory.h"
#include "planner/reference_line/reference_line_factory.h"
#include "visualization/visualization_manager.h"

class LocalPlannerNode : public rclcpp::Node {
 public:
  LocalPlannerNode();

 private:
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void planTimerCallback();

  // 找到最近路径点索引
  size_t findNearestIndex(const nav_msgs::msg::Path& path, double x, double y,
                          double theta) const;

  // 按 path_density_ 对局部路径进行等弧长插值，位置线性插值贴合全局轨迹，
  // 航向采用所属线段的原始方向，保留全局路径上的航向突变。
  nav_msgs::msg::Path interpolateLocalPath(const nav_msgs::msg::Path& path,
                                           size_t min_points) const;

  // 全局路径缓存
  nav_msgs::msg::Path global_path_;
  bool has_global_path_{false};

  // 当前位姿
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  geometry_msgs::msg::PoseWithCovariance current_pose_;
  geometry_msgs::msg::TwistWithCovariance current_twist_;
  bool has_pose_{false};

  // 参数
  double local_path_length_;             // 局部路径长度 (m)
  double max_nearest_search_distance_;   // 最近点向前搜索最大弧长 (m)
  double path_density_;                  // 局部路径插值密度 (points/m)
  double lookahead_distance_;            // 纯追踪前视距离 (m)
  double max_linear_speed_;              // 最大线速度 (m/s)
  double max_angular_speed_;             // 最大角速度 (rad/s)
  double goal_tolerance_;                // 目标容忍距离 (m)

  std::shared_ptr<map::StaticMap> map_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      dwa_vis_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      traj_vis_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<reference_line::ReferenceLineBase> reference_line_;
  std::unique_ptr<local_planner::LocalPlannerBase> local_planner_ = nullptr;

  std::once_flag flag;

  size_t last_nearest_idx_ = 0;
};
