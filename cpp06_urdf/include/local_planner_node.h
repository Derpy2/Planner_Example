#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode();

private:
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void planTimerCallback();

    // 平滑局部路径（简单移动平均）
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);

    // 找到最近路径点索引
    size_t findNearestIndex(const nav_msgs::msg::Path& path, double x, double y) const;

    // 四元数转 yaw
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

    // 全局路径缓存
    nav_msgs::msg::Path global_path_;
    bool has_global_path_{false};

    // 当前位姿
    double current_x_{0.0};
    double current_y_{0.0};
    double current_yaw_{0.0};
    bool has_pose_{false};

    // 参数
    double local_path_length_;   // 局部路径长度 (m)
    double lookahead_distance_;  // 纯追踪前视距离 (m)
    double max_linear_speed_;    // 最大线速度 (m/s)
    double max_angular_speed_;   // 最大角速度 (rad/s)
    double goal_tolerance_;      // 目标容忍距离 (m)

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LOCAL_PLANNER_NODE_H
