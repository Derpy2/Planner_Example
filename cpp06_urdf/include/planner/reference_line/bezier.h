#pragma once

#include <vector>

#include "reference_line_base.h"

namespace reference_line {
class BezierReferenceLine : public ReferenceLineBase {
 public:
  BezierReferenceLine() {};

  BezierReferenceLine(const int k) : k_(k) {};

  nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path) override;

 private:
  std::vector<geometry_msgs::msg::PoseStamped> GetBezierCurve(
      const std::vector<geometry_msgs::msg::PoseStamped>& control_points);

  int k_ = 3;              // 贝塞尔曲线的阶数
  double init_scale_ = 5;  // 拉伸系数
};
}  // namespace reference_line