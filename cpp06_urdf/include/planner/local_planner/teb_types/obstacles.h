#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "common/geometry.h"

namespace teb_local_planner {

class Obstacle {
 public:
  enum ObstacleType { POINT, LINE, POLYGON };

  explicit Obstacle(ObstacleType type) : type_(type) {}
  virtual ~Obstacle() = default;

  ObstacleType type() const { return type_; }

  virtual double minDistance(const Eigen::Vector2d& point) const = 0;

  virtual bool isDynamic() const { return false; }

  virtual Eigen::Vector2d getCentroid() const = 0;

 protected:
  ObstacleType type_;
};

class PointObstacle : public Obstacle {
 public:
  PointObstacle(double x, double y) : Obstacle(POINT), pos_(x, y) {}

  double minDistance(const Eigen::Vector2d& point) const override {
    return (pos_ - point).norm();
  }

  Eigen::Vector2d getCentroid() const override { return pos_; }

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

  Eigen::Vector2d getCentroid() const override {
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto& p : polygon_) centroid += Eigen::Vector2d(p.x, p.y);
    if (!polygon_.empty())
      centroid /= static_cast<double>(polygon_.size());
    return centroid;
  }

  const common::Polygon2D& polygon() const { return polygon_; }

 private:
  common::Polygon2D polygon_;
};

typedef std::shared_ptr<Obstacle> ObstaclePtr;

}  // namespace teb_local_planner
