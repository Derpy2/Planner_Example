#include <cmath>

#include "planner/reference_line/slide_window.h"


namespace reference_line {

nav_msgs::msg::Path SlideWindowReferenceLine::smoothPath(const nav_msgs::msg::Path& path) {
    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;
    if (path.poses.empty()) {
        return smoothed_path;
    }
    // 使用滑动窗口进行平滑处理
    const size_t window_size = 5; // 窗口大小
    const size_t half_window = window_size / 2;
    const size_t path_size = path.poses.size();
    for (size_t i = 0; i < path_size; ++i) {
        geometry_msgs::msg::PoseStamped smoothed_pose;
        smoothed_pose.header = path.poses[i].header;
        double sum_x = 0.0;
        double sum_y = 0.0;
        size_t count = 0;
        // 累加窗口内的点
        for (int j = -static_cast<int>(half_window); j <= static_cast<int>(half_window); ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(path_size)) {
                sum_x += path.poses[idx].pose.position.x;
                sum_y += path.poses[idx].pose.position.y;
                ++count;
            }
        }
        // 计算平均值
        smoothed_pose.pose.position.x = sum_x / count;
        smoothed_pose.pose.position.y = sum_y / count;
        smoothed_pose.pose.position.z = path.poses[i].pose.position.z; // 保持原始高度
        // 保持原始朝向
        smoothed_pose.pose.orientation = path.poses[i].pose.orientation;
        smoothed_path.poses.push_back(smoothed_pose);
    }
    return smoothed_path;
}
} // namespace reference_line