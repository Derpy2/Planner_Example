#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "common/geometry.h"

namespace teb_local_planner {

typedef std::shared_ptr<Obstacle> ObstaclePtr;

class Obstacle {
 public:
  enum ObstacleType { POINT, LINE, POLYGON };

  explicit Obstacle(ObstacleType type) : type_(type) {}
  virtual ~Obstacle() = default;

  ObstacleType type() const { return type_; }

  virtual double minDistance(const Eigen::Vector2d& point) const = 0;

 protected:
  ObstacleType type_;
};

class PointObstacle : public Obstacle {
 public:
  PointObstacle(double x, double y) : Obstacle(POINT), pos_(x, y) {}

  double minDistance(const Eigen::Vector2d& point) const override {
    return (pos_ - point).norm();
  }

  const Eigen::Vector2d& position() const { return pos_; }

 private:
  Eigen::Vector2d pos_;
};

class PolygonObstacle : public Obstacle {
 public:
  explicit PolygonObstacle(const common::Polygon2D& polygon)
      : Obstacle(POLYGON), polygon_(polygon) {}

  double minDistance(const Eigen::Vector2d& point) const override {
    common::Node2D pt(point.x(), point.y());
    return common::pointToPolygonDistance(pt, polygon_);
  }

  const common::Polygon2D& polygon() const { return polygon_; }

 private:
  common::Polygon2D polygon_;
};

}  // namespace teb_local_planner
