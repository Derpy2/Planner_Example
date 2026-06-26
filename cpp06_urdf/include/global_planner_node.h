#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>

class GlobalPlannerNode : public rclcpp::Node
{
public:
    GlobalPlannerNode();

private:
    void buildMap();
    void addObstacle(double xmin, double ymin, double xmax, double ymax);
    inline int idx(int x, int y) const { return y * (int)width_ + x; }
    inline bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < (int)width_ && y < (int)height_;
    }
    void worldToGrid(double wx, double wy, int &gx, int &gy) const;
    void gridToWorld(int gx, int gy, double &wx, double &wy) const;
    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void planAndPublish(double sx, double sy, double gx, double gy);
    void publishRobotPose();
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
    static void setYaw(geometry_msgs::msg::Quaternion &q, double yaw);

private:
    struct AStarNode {
        int x, y;
        double f;
        bool operator>(const AStarNode &o) const {return f > o.f; }
    };

    double resolution_;
    unsigned int width_, height_;
    double origin_x_, origin_y_;
    nav_msgs::msg::OccupancyGrid map_msg_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Pose current_pose_;
    rclcpp::Time last_update_time_;

    bool has_start_{false};
    double start_x_{0.0}, start_y_{0.0};
};


