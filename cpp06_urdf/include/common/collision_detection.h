#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

#include "map/static_map.h"
#include "node3d.h"

namespace common {

namespace {
inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}  // namespace

class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template <typename T>
  bool isTraversable(const T* node) const {
    /* Depending on the used collision checking mechanism this needs to be
       adjusted standard: collision checking using the spatial occupancy
       enumeration other: collision checking using the 2d costmap and the
       navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    cost = configurationTest(x, y, t) ? 0 : 1;

    return cost <= 0;
  }

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t) const;

  std::vector<Node3D> GetVehiclePolygon(float x, float y, float t,
                                        float width, float length);

 private:
  std::shared_ptr<map::StaticMap> map_;
};

}  // namespace common