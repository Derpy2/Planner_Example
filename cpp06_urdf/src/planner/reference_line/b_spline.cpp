#include "planner/reference_line/b_spline.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace {
    constexpr double EPSILON = 1e-6;
}

namespace reference_line {

std::vector<double> BSplineReferenceLine::buildKNot(const int n, const int p, const int type) {
    std::vector<double> knots;
    knots.reserve(n + p + 1);
    size_t knots_size = n + p + 1;
    if (type == 1) {
        // 均匀B样条节点向量
        const double base = knots_size - 1;
        for (size_t i = 0; i < knots_size; ++i) {
            knots.emplace_back(static_cast<double>(i) / base);
        }
    } else if (type == 2) {
        // 准均匀B样条节点向量
        const double base = n - p + 1;
        for (size_t i = 0; i < knots_size; ++i) {
            if (i < static_cast<size_t>(p + 1)) {
                knots.emplace_back(0.0);
            } else if (i > static_cast<size_t>(n)) {
                knots.emplace_back(1.0);
            } else {
                knots.emplace_back(static_cast<double>(i - p) / base);
            }
        }
    }
    return knots;
}

double BSplineReferenceLine::bsplineBaseFunc(const int i, const int p, const std::vector<double>& knots, const double t) {
    if (p == 0) {
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    } else {
        double result = 0.0;
        result += (knots[i + p] - knots[i] > EPSILON) ? (t - knots[i]) / (knots[i + p] - knots[i]) * bsplineBaseFunc(i, p - 1, knots, t) : 0.0;
        result += (knots[i + p + 1] - knots[i + 1] > EPSILON) ? (knots[i + p + 1] - t) / (knots[i + p + 1] - knots[i + 1]) * bsplineBaseFunc(i + 1, p - 1, knots, t) : 0.0;
        return result; 
    }
    return 0.0;
}

nav_msgs::msg::Path BSplineReferenceLine::smoothPath(const nav_msgs::msg::Path& path) {
    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = path.header;
    if (path.poses.empty()) {
        return smoothed_path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> pose_points;
    for (const auto& pose_stamped : path.poses) {
        pose_points.push_back(pose_stamped);
    }
    const int pose_size = path.poses.size();
    const std::vector<double> knots = buildKNot(pose_size - 1, 3, type_);
    for (double i = 0.0; i <= 1.0; i += interval_) {
        geometry_msgs::msg::PoseStamped smoothed_pose;
        smoothed_pose.header = path.header;
        smoothed_pose.pose.position.x = 0.0;
        smoothed_pose.pose.position.y = 0.0;
        for (int j = 0; j < pose_size; ++j) {
            double basis = bsplineBaseFunc(j, 3, knots, i);
            smoothed_pose.pose.position.x += basis * pose_points[j].pose.position.x;
            smoothed_pose.pose.position.y += basis * pose_points[j].pose.position.y;
        }
        smoothed_pose.pose.position.z = path.poses[0].pose.position.z; // 保持原始高度
        smoothed_pose.pose.orientation = path.poses[0].pose.orientation; // 保持
        smoothed_path.poses.emplace_back(smoothed_pose);
    }

    return smoothed_path;
}

} // namespace reference_line