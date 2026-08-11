#pragma once

#include <Eigen/Core>
#include <cmath>

namespace teb_local_planner {

class PoseSE2 {
 public:
  PoseSE2() : position_(Eigen::Vector2d::Zero()), theta_(0.0) {}

  PoseSE2(double x, double y, double theta)
      : position_(x, y), theta_(theta) {}

  PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
      : position_(position), theta_(theta) {}

  void setZero() {
    position_.setZero();
    theta_ = 0.0;
  }

  void plus(const double* update) {
    position_.x() += update[0];
    position_.y() += update[1];
    theta_ += update[2];
    theta_ = std::fmod(theta_, 2.0 * M_PI);
    if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    if (theta_ < -M_PI) theta_ += 2.0 * M_PI;
  }

  inline Eigen::Vector2d& position() { return position_; }
  inline const Eigen::Vector2d& position() const { return position_; }

  inline double& x() { return position_.x(); }
  inline const double& x() const { return position_.x(); }

  inline double& y() { return position_.y(); }
  inline const double& y() const { return position_.y(); }

  inline double& theta() { return theta_; }
  inline const double& theta() const { return theta_; }

 private:
  Eigen::Vector2d position_;
  double theta_;
};

}  // namespace teb_local_planner
