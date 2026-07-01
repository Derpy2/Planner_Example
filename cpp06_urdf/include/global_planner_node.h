#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "../include/map/static_map.h"
#include "../include/planner/global_planner/global_planner_factory.h"

class GlobalPlannerNode : public rclcpp::Node {
 public:
  GlobalPlannerNode();

 private:
  void startCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void planAndPublish(const geometry_msgs::msg::Pose& start,
                      const geometry_msgs::msg::Pose goal);
  void publishRobotPose();
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
  static void setYaw(geometry_msgs::msg::Quaternion& q, double yaw);

 private:
  std::shared_ptr<map::StaticMap> map_;
  std::unique_ptr<global_planner::GlobalPlannerBase> global_planner_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::Time last_update_time_;

  std::once_flag flag;

  bool has_start_{false};
  double start_x_{0.0}, start_y_{0.0};
};
