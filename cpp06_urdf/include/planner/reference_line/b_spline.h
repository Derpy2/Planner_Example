#pragma once

#include "reference_line_base.h"

namespace reference_line {
class BSplineReferenceLine : public ReferenceLineBase {
public:
    BSplineReferenceLine() {};

    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path) override;
private:
    std::vector<double> buildKNot(const int n, const int p, const int type);

    double bsplineBaseFunc(const int i, const int p, const std::vector<double>& knots, const double t);

    int type_ = 2; // 1: 均匀B样条, 2: 准均匀B样条
    double interval_ = 0.01; // 插值间隔
};
} // namespace reference_line