// Local trajectory planner node
// Subscribes /global_path and /odom
// Publishes /local_path and /cmd_vel
#include "../include/local_planner_node.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "common/util.h"
#include "planner/local_planner/dwa.h"

LocalPlannerNode::LocalPlannerNode() : Node("local_planner_node") {
  local_path_length_ =
      this->declare_parameter<double>("local_path_length", 2.0);
  max_nearest_search_distance_ =
      this->declare_parameter<double>("max_nearest_search_distance", 1.5);
  path_density_ = this->declare_parameter<double>("path_density", 8.0);
  lookahead_distance_ =
      this->declare_parameter<double>("lookahead_distance", 0.6);
  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.4);
  max_angular_speed_ =
      this->declare_parameter<double>("max_angular_speed", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.3);

  map_ = std::make_shared<map::StaticMap>();
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
  dwa_vis_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "dwa_sampled_trajectories", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&LocalPlannerNode::planTimerCallback, this));

  reference_line_ =
      reference_line::ReferenceLineFactory::GetReferenceLineCreator(
          reference_line::ReferenceLineType::SlideWindow);

  RCLCPP_INFO(this->get_logger(),
              "Local planner started. lookahead=%.2f, max_v=%.2f, max_w=%.2f, "
              "max_nearest_search_dist=%.2f, path_density=%.2f",
              lookahead_distance_, max_linear_speed_, max_angular_speed_,
              max_nearest_search_distance_, path_density_);
}

void LocalPlannerNode::globalPathCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  global_path_ = *msg;

  has_global_path_ = !msg->poses.empty();
  RCLCPP_INFO(this->get_logger(), "Received global path with %zu poses.",
              msg->poses.size());
  last_nearest_idx_ = 0;
  if (local_planner_ != nullptr) {
    local_planner_->init();
  }
}

void LocalPlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_yaw_ = common::yawFromQuaternion(msg->pose.pose.orientation);
  current_pose_ = msg->pose;
  current_twist_ = msg->twist;
  has_pose_ = true;
}

size_t LocalPlannerNode::findNearestIndex(const nav_msgs::msg::Path& path,
                                          double x, double y,
                                          double theta) const {
  if (path.poses.empty()) {
    return 0;
  }

  size_t nearest = last_nearest_idx_;
  double min_dist = std::numeric_limits<double>::infinity();
  double accumulated_dist = 0.0;

  for (size_t i = last_nearest_idx_; i < path.poses.size(); ++i) {
    // 限制从上次最近点开始向前搜索的最大弧长，防止全局路径密集时
    // 最近点跳到远离当前参考线起点的位置
    if (i > last_nearest_idx_) {
      double dx_seg =
          path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      double dy_seg =
          path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
      accumulated_dist += std::hypot(dx_seg, dy_seg);
      if (accumulated_dist > max_nearest_search_distance_) {
        break;
      }
    }

    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    // 只考虑车体前方（或侧方）的点
    if (dx * std::cos(theta) + dy * std::sin(theta) < 0) {
      continue;
    }
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = i;
    }
  }
  return nearest;
}

nav_msgs::msg::Path LocalPlannerNode::interpolateLocalPath(
    const nav_msgs::msg::Path& path, size_t min_points) const {
  if (path.poses.size() < 2) {
    return path;
  }

  // 计算累积弧长
  std::vector<double> cum_lengths(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    double dx =
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
    double dy =
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
    cum_lengths[i] = cum_lengths[i - 1] + std::hypot(dx, dy);
  }
  const double total_length = cum_lengths.back();
  if (total_length < 1e-6) {
    return path;  // 所有点重合，无法插值
  }

  // 计算各原始线段的航向角；插值点采用所属线段的航向，
  // 这样在全局路径发生航向突变的位置，参考线也会保持突变。
  std::vector<double> segment_yaws(path.poses.size() - 1);
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    segment_yaws[i] = common::computeYaw(path.poses[i], path.poses[i + 1]);
  }

  // 按给定密度计算目标点数：8 points/m，即每隔 0.125m 一个点
  const double spacing = 1.0 / path_density_;
  size_t target_count =
      static_cast<size_t>(std::ceil(total_length / spacing)) + 1;
  if (target_count < min_points) {
    target_count = min_points;
  }

  nav_msgs::msg::Path result;
  result.header = path.header;
  const double step_length =
      total_length / static_cast<double>(target_count - 1);

  for (size_t k = 0; k < target_count; ++k) {
    const double s = std::min(k * step_length, total_length);

    // 定位 s 所在的原始线段
    size_t idx = 0;
    for (size_t i = 1; i < cum_lengths.size(); ++i) {
      if (s <= cum_lengths[i]) {
        idx = i - 1;
        break;
      }
    }

    const double seg_len = cum_lengths[idx + 1] - cum_lengths[idx];
    double ratio = 0.0;
    if (seg_len > 1e-6) {
      ratio = (s - cum_lengths[idx]) / seg_len;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    // 位置线性插值，插值点严格落在全局路径的原始线段上
    pose.pose.position.x = (1.0 - ratio) * path.poses[idx].pose.position.x +
                           ratio * path.poses[idx + 1].pose.position.x;
    pose.pose.position.y = (1.0 - ratio) * path.poses[idx].pose.position.y +
                           ratio * path.poses[idx + 1].pose.position.y;
    pose.pose.position.z = (1.0 - ratio) * path.poses[idx].pose.position.z +
                           ratio * path.poses[idx + 1].pose.position.z;

    // 直接使用所属原始线段的航向，保留航向突变
    const double yaw = segment_yaws[idx];
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    result.poses.push_back(pose);
  }

  return result;
}

void LocalPlannerNode::planTimerCallback() {
  if (!has_global_path_ || !has_pose_) {
    return;
  }

  size_t nearest_idx =
      findNearestIndex(global_path_, current_x_, current_y_, current_yaw_);
  if (nearest_idx < last_nearest_idx_) {
    nearest_idx = last_nearest_idx_;
  }
  last_nearest_idx_ = nearest_idx;

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

  // 点数不足时进行插值，保证后续平滑与控制器有足够路径点
  local_path = interpolateLocalPath(local_path, 5);

  if (local_path.poses.size() < 5) {
    return;
  }

  std::call_once(flag, [&]() {
    this->local_planner_ =
        local_planner::LocalPlannerFactory::CreateLocalPlanner(
            local_planner::LocalPlannerType::DWA, map_, get_logger());
    local_planner_->init();
  });

  nav_msgs::msg::Path smoothed_local = reference_line_->smoothPath(local_path);
  smoothed_local.header = local_path.header;
  local_path_pub_->publish(smoothed_local);

  local_planner_->setCurrentPose(current_pose_);
  local_planner_->setCurrentTwist(current_twist_);
  local_planner_->setSmoothedPath(smoothed_local);
  geometry_msgs::msg::Twist cmd = local_planner_->getControlCmd();
  RCLCPP_INFO(this->get_logger(), "Cmd: v=%.2f, omega=%.2f", cmd.linear.x,
              cmd.angular.z);
  cmd_vel_pub_->publish(cmd);

  local_planner_->visualizeSampledTrajectories("map");

  auto markers =
      visualization::VisualizationManager::Instance().GetAllMarkers();
  if (!markers.markers.empty()) {
    dwa_vis_marker_pub_->publish(markers);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
