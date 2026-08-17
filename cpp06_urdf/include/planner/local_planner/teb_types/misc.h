#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>

namespace teb_local_planner {

enum class RotType { left, none, right };

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

inline double fast_sigmoid(double x) { return x / (1.0 + std::abs(x)); }

template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2) {
  return v1.x() * v2.y() - v2.x() * v1.y();
}

inline int g2o_sign(int x) { return (x > 0) ? 1 : ((x < 0) ? -1 : 0); }

inline double g2o_sign(double x) {
  return (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0);
}

#ifndef TEB_ASSERT_MSG
#define TEB_ASSERT_MSG(cond, fmt, ...)                         \
  do {                                                         \
    if (!(cond)) {                                             \
      fprintf(stderr, "TEB_ASSERT: " fmt "\n", ##__VA_ARGS__); \
    }                                                          \
  } while (0)
#endif

}  // namespace teb_local_planner
