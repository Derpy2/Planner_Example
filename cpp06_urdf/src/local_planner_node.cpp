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
  lookahead_distance_ =
      this->declare_parameter<double>("lookahead_distance", 0.6);
  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.4);
  max_angular_speed_ =
      this->declare_parameter<double>("max_angular_speed", 1.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.15);

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
              "Local planner started. lookahead=%.2f, max_v=%.2f, max_w=%.2f",
              lookahead_distance_, max_linear_speed_, max_angular_speed_);
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
  size_t nearest = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = last_nearest_idx_; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - x;
    double dy = path.poses[i].pose.position.y - y;
    if (dx * std::cos(theta) + dy * std::sin(theta) < 0) {
      continue;
    }
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = i;
    }
    // if (dist > lookahead_distance_) {
    //   nearest = i;
    //   break;
    // }
  }
  return nearest;
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
