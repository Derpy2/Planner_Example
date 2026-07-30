#pragma once

namespace global_planner {

struct ReedsSheppPath {
  double qi[3];     // initial configuration [x, y, yaw]
  double param[6];  // lengths of the segments (signed: positive = forward, negative = backward)
  int seg_type[6];  // 0 = left, 1 = straight, 2 = right
  int segments;     // number of valid segments
  double rho;       // turning radius
  double length;    // total absolute length
};

// Compute the shortest Reeds-Shepp path from q0 to q1 with turning radius rho.
// Returns 0 on success, non-zero on failure.
int reeds_shepp_init(double q0[3], double q1[3], double rho, ReedsSheppPath* path);

// Return the total absolute length of the path.
double reeds_shepp_path_length(ReedsSheppPath* path);

// Sample a configuration along the path at distance t from the start.
// Returns 0 on success, non-zero if t is out of bounds.
int reeds_shepp_path_sample(ReedsSheppPath* path, double t, double q[3]);

}  // namespace global_planner
