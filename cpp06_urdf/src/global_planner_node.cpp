// Global path planner node based on A*
// Subscribes /initialpose as start, /goal_pose as goal, /cmd_vel as velocity
// command Publishes /map (OccupancyGrid), /global_path (Path), /odom (Odometry)
#include "../include/global_planner_node.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "visualization/visualization_manager.h"

GlobalPlannerNode::GlobalPlannerNode() : Node("global_planner_node") {
  map_ = std::make_shared<map::StaticMap>();

  // global_planner_ =
  // global_planner::GlobalPlannerFactory::CreateGlobalPlanner(
  //     global_planner::HYBRID_A_STAR, map_, get_logger());

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local());
  path_pub_ = create_publisher<nav_msgs::msg::Path>("global_path", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker_array", 10);
  start_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 10,
          std::bind(&GlobalPlannerNode::startCallback, this,
                    std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&GlobalPlannerNode::goalCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&GlobalPlannerNode::cmdVelCallback, this,
                std::placeholders::_1));
  map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    map_pub_->publish(map_->getMapMsg());
  });
  has_start_ = true;
  start_x_ = 0.0;
  start_y_ = 0.0;
  current_pose_.position.z = 0.0;
  current_pose_.orientation.w = 1.0;
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  pose_timer_ =
      create_wall_timer(std::chrono::milliseconds(50),
                        std::bind(&GlobalPlannerNode::publishRobotPose, this));
  vis_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        auto markers = visualization::VisualizationManager::Instance().GetAllMarkers();
        if (!markers.markers.empty()) {
          vis_pub_->publish(markers);
        }
      });
  last_update_time_ = now();
  RCLCPP_INFO(get_logger(),
              "Global planner started. Use '2D Pose Estimate' for start and "
              "'2D Goal Pose' for goal in RViz.");
}

void GlobalPlannerNode::startCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  start_x_ = msg->pose.pose.position.x;
  start_y_ = msg->pose.pose.position.y;
  current_pose_ = msg->pose.pose;
  current_pose_.position.z = 0.0;
  has_start_ = true;
  last_update_time_ = now();
  RCLCPP_INFO(get_logger(), "Got start: (%.2f, %.2f)", start_x_, start_y_);
  publishRobotPose();
}

void GlobalPlannerNode::goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!has_start_) {
    RCLCPP_WARN(get_logger(), "No start set yet, ignoring goal.");
    return;
  }
  double gx = msg->pose.position.x;
  double gy = msg->pose.position.y;
  RCLCPP_INFO(get_logger(), "Got goal: (%.2f, %.2f), planning...", gx, gy);
  planAndPublish(current_pose_, msg->pose);
}

void GlobalPlannerNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
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

void GlobalPlannerNode::planAndPublish(const geometry_msgs::msg::Pose& start,
                                       const geometry_msgs::msg::Pose goal) {
  std::call_once(flag, [&]() {
    this->global_planner_ =
        global_planner::GlobalPlannerFactory::CreateGlobalPlanner(
            global_planner::HYBRID_A_STAR, map_, get_logger());
  });

  global_planner_->setStartPose(start);
  global_planner_->setGoalPose(goal);
  nav_msgs::msg::Path path = global_planner_->searchPath();
  path.header.frame_id = "map";
  path.header.stamp = now();
  path_pub_->publish(path);
  RCLCPP_INFO(get_logger(), "Path published with %zu poses.",
              path.poses.size());
}

void GlobalPlannerNode::publishRobotPose() {
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

double GlobalPlannerNode::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void GlobalPlannerNode::setYaw(geometry_msgs::msg::Quaternion& q, double yaw) {
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
