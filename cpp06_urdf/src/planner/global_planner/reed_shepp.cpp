#include "planner/global_planner/reed_shepp.h"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <cmath>
#include <iostream>

namespace global_planner {

namespace {

using SE2State = ompl::base::SE2StateSpace::StateType;
using RSPath = ompl::base::ReedsSheppStateSpace::ReedsSheppPath;
using SegmentType = ompl::base::ReedsSheppStateSpace::ReedsSheppPathSegmentType;

double normalizeAngle(double theta) {
  while (theta < 0.0) theta += 2.0 * M_PI;
  while (theta >= 2.0 * M_PI) theta -= 2.0 * M_PI;
  return theta;
}

}  // namespace

int reeds_shepp_init(double q0[3], double q1[3], double rho,
                     ReedsSheppPath* path) {
  if (rho <= 0.0 || path == nullptr) {
    return -1;
  }

  ompl::base::ReedsSheppStateSpace rs_space(rho);
  SE2State* from = static_cast<SE2State*>(rs_space.allocState());
  SE2State* to = static_cast<SE2State*>(rs_space.allocState());

  from->setXY(q0[0], q0[1]);
  from->setYaw(q0[2]);
  to->setXY(q1[0], q1[1]);
  to->setYaw(q1[2]);

  RSPath rs_path = rs_space.reedsShepp(from, to);

  for (int i = 0; i < 3; ++i) {
    path->qi[i] = q0[i];
  }
  path->rho = rho;
  path->length = rs_path.length() * rho;
  path->segments = 0;

  for (int i = 0; i < 5; ++i) {
    if (std::abs(rs_path.length_[i]) < 1e-10) {
      continue;
    }

    int seg_type = -1;
    switch (rs_path.type_[i]) {
      case SegmentType::RS_LEFT:
        seg_type = 0;
        break;
      case SegmentType::RS_STRAIGHT:
        seg_type = 1;
        break;
      case SegmentType::RS_RIGHT:
        seg_type = 2;
        break;
      default:
        continue;
    }

    path->param[path->segments] = rs_path.length_[i];
    path->seg_type[path->segments] = seg_type;
    path->segments++;
  }

  rs_space.freeState(from);
  rs_space.freeState(to);

  return path->segments > 0 ? 0 : -1;
}

double reeds_shepp_path_length(ReedsSheppPath* path) {
  if (path == nullptr) {
    return 0.0;
  }
  double length = 0.0;
  for (int i = 0; i < path->segments; ++i) {
    length += std::abs(path->param[i]);
  }
  return length * path->rho;
}

int reeds_shepp_path_sample(ReedsSheppPath* path, double t, double q[3]) {
  if (path == nullptr || path->segments == 0) {
    return -1;
  }

  double total_length = reeds_shepp_path_length(path);
  if (t < 0.0 || t > total_length) {
    return -1;
  }

  double s = t / path->rho;
  double x = path->qi[0];
  double y = path->qi[1];
  double theta = path->qi[2];

  for (int i = 0; i < path->segments; ++i) {
    double seg_len = std::abs(path->param[i]);
    if (s > seg_len && i < path->segments - 1) {
      double signed_len = path->param[i];
      switch (path->seg_type[i]) {
        case 0:  // Left turn
          x += path->rho * (std::sin(theta + signed_len) - std::sin(theta));
          y -= path->rho * (std::cos(theta + signed_len) - std::cos(theta));
          theta += signed_len;
          break;
        case 1:  // Straight
          x += path->rho * std::cos(theta) * signed_len;
          y += path->rho * std::sin(theta) * signed_len;
          break;
        case 2:  // Right turn
          x -= path->rho * (std::sin(theta - signed_len) - std::sin(theta));
          y += path->rho * (std::cos(theta - signed_len) - std::cos(theta));
          theta -= signed_len;
          break;
      }
      s -= seg_len;
      continue;
    }

    double partial = std::min(s, seg_len);
    double signed_partial = (path->param[i] >= 0.0 ? 1.0 : -1.0) * partial;

    switch (path->seg_type[i]) {
      case 0:  // Left turn
        q[0] = x +
               path->rho * (std::sin(theta + signed_partial) - std::sin(theta));
        q[1] = y -
               path->rho * (std::cos(theta + signed_partial) - std::cos(theta));
        q[2] = theta + signed_partial;
        break;
      case 1:  // Straight
        q[0] = x + path->rho * std::cos(theta) * signed_partial;
        q[1] = y + path->rho * std::sin(theta) * signed_partial;
        q[2] = theta;
        break;
      case 2:  // Right turn
        q[0] = x -
               path->rho * (std::sin(theta - signed_partial) - std::sin(theta));
        q[1] = y +
               path->rho * (std::cos(theta - signed_partial) - std::cos(theta));
        q[2] = theta - signed_partial;
        break;
    }

    q[2] = normalizeAngle(q[2]);
    return 0;
  }

  q[0] = x;
  q[1] = y;
  q[2] = normalizeAngle(theta);
  return 0;
}

}  // namespace global_planner
