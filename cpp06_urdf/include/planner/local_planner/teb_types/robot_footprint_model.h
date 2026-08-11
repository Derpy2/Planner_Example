#pragma once

#include <Eigen/Core>
#include <memory>

#include "planner/local_planner/teb_types/obstacles.h"
#include "planner/local_planner/teb_types/pose_se2.h"

namespace teb_local_planner {

class BaseRobotFootprintModel {
 public:
  virtual ~BaseRobotFootprintModel() = default;

  virtual double calculateDistance(const PoseSE2& pose,
                                   const Obstacle* obstacle) const = 0;

  void setRobotRadius(double radius) { radius_ = radius; }
  double robotRadius() const { return radius_; }

 protected:
  double radius_ = 0.3;
};

class CircularRobotFootprintModel : public BaseRobotFootprintModel {
 public:
  double calculateDistance(const PoseSE2& pose,
                           const Obstacle* obstacle) const override {
    double dist_to_obs = obstacle->minDistance(pose.position());
    return std::max(0.0, dist_to_obs - radius_);
  }
};

}  // namespace teb_local_planner
