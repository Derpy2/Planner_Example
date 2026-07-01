// Local trajectory planner node
// Subscribes /global_path and /odom
// Publishes /local_path and /cmd_vel
#include "../include/local_planner_node.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "common/util.h"

LocalPlannerNode::LocalPlannerNode() : Node("local_planner_node") {
  local_path_length_ =
      this->declare_parameter<double>("local_path_length", 2.0);
  lookahead_distance_ =
      this->declare_parameter<double>("lookahead_distance", 0.6);
  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.4);
  max_angular_speed_ =
      this->declare_parameter<double>("max_angular_speed", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.15);

  global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "global_path", 10,
      std::bind(&LocalPlannerNode::globalPathCallback, this,
                std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));

  local_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LocalPlannerNode::planTimerCallback, this));

  reference_line_ =
      reference_line::ReferenceLineFactory::GetReferenceLineCreator(
          reference_line::ReferenceLineType::BSpline);

  RCLCPP_INFO(this->get_logger(),
              "Local planner started. lookahead=%.2f, max_v=%.2f, max_w=%.2f",
              lookahead_distance_, max_linear_speed_, max_angular_speed_);
}

void LocalPlannerNode::globalPathCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  global_path_ = *msg;
  has_global_path_ = !msg->poses.empty();
  RCLCPP_INFO(this->get_logger(), "Received global path with %zu poses.",
              msg->poses.size());
}

void LocalPlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_yaw_ = common::yawFromQuaternion(msg->pose.pose.orientation);
  current_pose_ = msg->pose;
  has_pose_ = true;
}

size_t LocalPlannerNode::findNearestIndex(const nav_msgs::msg::Path& path,
                                          double x, double y) const {
  size_t nearest = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = i;
    }
  }
  return nearest;
}

void LocalPlannerNode::planTimerCallback() {
  if (!has_global_path_ || !has_pose_) {
    return;
  }

  size_t nearest_idx = findNearestIndex(global_path_, current_x_, current_y_);

  // 检查是否到达终点
  auto goal_pose = global_path_.poses.back().pose;
  double dist_to_goal = std::hypot(goal_pose.position.x - current_x_,
                                   goal_pose.position.y - current_y_);
  if (dist_to_goal < goal_tolerance_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
    return;
  }

  // 裁剪局部窗口
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = global_path_.header.frame_id;
  local_path.header.stamp = this->now();
  local_path.poses.push_back(global_path_.poses[nearest_idx]);
  double accumulated_length = 0.0;
  for (size_t i = nearest_idx + 1; i < global_path_.poses.size(); ++i) {
    double dx = global_path_.poses[i].pose.position.x -
                global_path_.poses[i - 1].pose.position.x;
    double dy = global_path_.poses[i].pose.position.y -
                global_path_.poses[i - 1].pose.position.y;
    accumulated_length += std::hypot(dx, dy);
    if (accumulated_length > local_path_length_) {
      break;
    }
    local_path.poses.push_back(global_path_.poses[i]);
  }

  if (local_path.poses.size() < 2) {
    return;
  }

  nav_msgs::msg::Path smoothed_local = reference_line_->smoothPath(local_path);
  smoothed_local.header = local_path.header;

  // 计算每个点的朝向并发布局部路径
  for (size_t i = 0; i < smoothed_local.poses.size(); ++i) {
    double yaw = 0.0;
    if (i < smoothed_local.poses.size() - 1) {
      double dx = smoothed_local.poses[i + 1].pose.position.x -
                  smoothed_local.poses[i].pose.position.x;
      double dy = smoothed_local.poses[i + 1].pose.position.y -
                  smoothed_local.poses[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      double dx = smoothed_local.poses[i].pose.position.x -
                  smoothed_local.poses[i - 1].pose.position.x;
      double dy = smoothed_local.poses[i].pose.position.y -
                  smoothed_local.poses[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    smoothed_local.poses[i].pose.orientation.x = 0.0;
    smoothed_local.poses[i].pose.orientation.y = 0.0;
    smoothed_local.poses[i].pose.orientation.z = std::sin(yaw / 2.0);
    smoothed_local.poses[i].pose.orientation.w = std::cos(yaw / 2.0);
  }
  local_path_pub_->publish(smoothed_local);

  // 纯追踪 (Pure Pursuit) 找前视点
  size_t target_idx = nearest_idx;
  for (size_t i = nearest_idx; i < smoothed_local.poses.size(); ++i) {
    double dx = smoothed_local.poses[i].pose.position.x - current_x_;
    double dy = smoothed_local.poses[i].pose.position.y - current_y_;
    double dist = std::hypot(dx, dy);
    if (dist >= lookahead_distance_) {
      target_idx = i;
      break;
    }
  }
  if (target_idx == nearest_idx &&
      nearest_idx + 1 < smoothed_local.poses.size()) {
    target_idx = smoothed_local.poses.size() - 1;
  }

  double target_x = smoothed_local.poses[target_idx].pose.position.x;
  double target_y = smoothed_local.poses[target_idx].pose.position.y;
  double dx = target_x - current_x_;
  double dy = target_y - current_y_;
  double target_yaw = std::atan2(dy, dx);
  double alpha = target_yaw - current_yaw_;
  // 归一化到 [-pi, pi]
  while (alpha > M_PI) alpha -= 2.0 * M_PI;
  while (alpha < -M_PI) alpha += 2.0 * M_PI;

  double L = std::hypot(dx, dy);
  if (L < 1e-3) L = lookahead_distance_;

  // 曲率控制
  double v = max_linear_speed_;
  // 大角度时减速
  v *= std::max(0.0, std::cos(alpha));
  if (v < 0.05) v = 0.05;

  double curvature = 2.0 * std::sin(alpha) / L;
  double omega = v * curvature;
  if (omega > max_angular_speed_) omega = max_angular_speed_;
  if (omega < -max_angular_speed_) omega = -max_angular_speed_;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = v;
  cmd.angular.z = omega;
  cmd_vel_pub_->publish(cmd);
}

// double LocalPlannerNode::yawFromQuaternion(
//     const geometry_msgs::msg::Quaternion& q) {
//   return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
//                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
// }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
