// Global path planner node based on A*
// Subscribes /initialpose as start, /goal_pose as goal, /cmd_vel as velocity command
// Publishes /map (OccupancyGrid), /global_path (Path), /odom (Odometry)
#include "../include/global_planner_node.h"

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

GlobalPlannerNode::GlobalPlannerNode() : Node("global_planner_node")
{
  resolution_ = 0.05;       // 0.05 m per cell
  width_  = 200;            // 10 m
  height_ = 200;
  origin_x_ = -5.0;
  origin_y_ = -5.0;
  buildMap();

  map_pub_  = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("global_path", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&GlobalPlannerNode::startCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&GlobalPlannerNode::cmdVelCallback, this, std::placeholders::_1));
  map_timer_ = create_wall_timer(std::chrono::seconds(1),
      [this]() { map_pub_->publish(map_msg_); });
  has_start_ = true;
  start_x_ = 0.0;
  start_y_ = 0.0;
  current_pose_.position.z = 0.0;
  current_pose_.orientation.w = 1.0;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  pose_timer_ = create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&GlobalPlannerNode::publishRobotPose, this));
  last_update_time_ = now();
  RCLCPP_INFO(get_logger(),
      "Global planner started. Use '2D Pose Estimate' for start and '2D Goal Pose' for goal in RViz.");
}

void GlobalPlannerNode::buildMap()
{
  map_msg_.header.frame_id = "map";
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width  = width_;
  map_msg_.info.height = height_;
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(width_ * height_, 0);
  // Rectangular obstacles (xmin, ymin, xmax, ymax)
  addObstacle(-2.0, -1.0,  1.0,  2.0);
  addObstacle( 1.0, -3.0,  2.0,  0.0);
  addObstacle(-3.5,  1.5, -1.5,  2.5);
  addObstacle( 2.5,  1.0,  3.5,  3.5);
  // Walls around the map
  for (int x = 0; x < (int)width_; ++x) {
    map_msg_.data[idx(x, 0)] = 100;
    map_msg_.data[idx(x, height_ - 1)] = 100;
  }
  for (int y = 0; y < (int)height_; ++y) {
    map_msg_.data[idx(0, y)] = 100;
    map_msg_.data[idx(width_ - 1, y)] = 100;
  }
}

void GlobalPlannerNode::addObstacle(double xmin, double ymin, double xmax, double ymax)
{
  int gx0, gy0, gx1, gy1;
  worldToGrid(xmin, ymin, gx0, gy0);
  worldToGrid(xmax, ymax, gx1, gy1);
  for (int y = gy0; y <= gy1; ++y) {
    for (int x = gx0; x <= gx1; ++x) {
      if (inBounds(x, y)) {
        map_msg_.data[idx(x, y)] = 100;
      }
    }
  }
}

void GlobalPlannerNode::worldToGrid(double wx, double wy, int &gx, int &gy) const {
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

void GlobalPlannerNode::gridToWorld(int gx, int gy, double &wx, double &wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

void GlobalPlannerNode::startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  start_x_ = msg->pose.pose.position.x;
  start_y_ = msg->pose.pose.position.y;
  current_pose_ = msg->pose.pose;
  current_pose_.position.z = 0.0;
  has_start_ = true;
  last_update_time_ = now();
  RCLCPP_INFO(get_logger(), "Got start: (%.2f, %.2f)", start_x_, start_y_);
  publishRobotPose();
}

void GlobalPlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!has_start_) {
    RCLCPP_WARN(get_logger(), "No start set yet, ignoring goal.");
    return;
  }
  double gx = msg->pose.position.x;
  double gy = msg->pose.position.y;
  RCLCPP_INFO(get_logger(), "Got goal: (%.2f, %.2f), planning...", gx, gy);
  planAndPublish(start_x_, start_y_, gx, gy);
}

void GlobalPlannerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto now_time = now();
  double dt = (now_time - last_update_time_).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    last_update_time_ = now_time;
    return;
  }
  last_update_time_ = now_time;

  double v = msg->linear.x;
  double w = msg->angular.z;
  double yaw = yawFromQuaternion(current_pose_.orientation);

  current_pose_.position.x += v * std::cos(yaw) * dt;
  current_pose_.position.y += v * std::sin(yaw) * dt;
  yaw += w * dt;
  setYaw(current_pose_.orientation, yaw);

  // Update start so replanning uses current position
  start_x_ = current_pose_.position.x;
  start_y_ = current_pose_.position.y;
}

void GlobalPlannerNode::planAndPublish(double sx, double sy, double gx, double gy) {
  int sgx, sgy, ggx, ggy;
  worldToGrid(sx, sy, sgx, sgy);
  worldToGrid(gx, gy, ggx, ggy);

  if (!inBounds(sgx, sgy) || !inBounds(ggx, ggy)) {
    RCLCPP_WARN(get_logger(), "Start or goal out of map bounds.");
    return;
  }
  if (map_msg_.data[idx(sgx, sgy)] >= 50 || map_msg_.data[idx(ggx, ggy)] >= 50) {
    RCLCPP_WARN(get_logger(), "Start or goal lies on an obstacle.");
    return;
  }
  const int N = (int)width_ * (int)height_;
  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(N, -1);
  std::vector<bool> closed(N, false);
  auto h = [&](int x, int y) {
    double dx = x - ggx, dy = y - ggy;
    return std::hypot(dx, dy);
  };
  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
  gscore[idx(sgx, sgy)] = 0.0;
  open.push({sgx, sgy, h(sgx, sgy)});
  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double costs[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
  bool found = false;
  while (!open.empty()) {
    AStarNode cur = open.top(); open.pop();
    int ci = idx(cur.x, cur.y);
    if (closed[ci]) continue;
    closed[ci] = true;
    if (cur.x == ggx && cur.y == ggy) { found = true; break; }
    for (int k = 0; k < 8; ++k) {
      int nx = cur.x + dxs[k];
      int ny = cur.y + dys[k];
      if (!inBounds(nx, ny)) continue;
      int ni = idx(nx, ny);
      if (map_msg_.data[ni] >= 50) continue;
      double tentative = gscore[ci] + costs[k] * resolution_;
      if (tentative < gscore[ni]) {
        gscore[ni] = tentative;
        came_from[ni] = ci;
        open.push({nx, ny, tentative + h(nx, ny) * resolution_});
      }
    }
  }
  if (!found) {
    RCLCPP_WARN(get_logger(), "A* failed to find a path.");
    return;
  }
  std::vector<int> rev;
  for (int cur = idx(ggx, ggy); cur != -1; cur = came_from[cur]) rev.push_back(cur);
  std::reverse(rev.begin(), rev.end());
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = now();
  path.poses.reserve(rev.size());
  for (int i : rev) {
    int x = i % (int)width_;
    int y = i / (int)width_;
    double wx, wy;
    gridToWorld(x, y, wx, wy);
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  path_pub_->publish(path);
  RCLCPP_INFO(get_logger(), "Path published with %zu poses.", path.poses.size());
}

void GlobalPlannerNode::publishRobotPose()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";
  t.transform.translation.x = current_pose_.position.x;
  t.transform.translation.y = current_pose_.position.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = current_pose_.orientation;
  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Odometry odom;
  odom.header = t.header;
  odom.child_frame_id = t.child_frame_id;
  odom.pose.pose = current_pose_;
  odom_pub_->publish(odom);
}

double GlobalPlannerNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void GlobalPlannerNode::setYaw(geometry_msgs::msg::Quaternion &q, double yaw)
{
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
