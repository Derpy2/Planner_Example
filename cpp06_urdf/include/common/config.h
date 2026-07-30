#pragma once

#include <cmath>
#include <string>
namespace common {

namespace constants {
/// =============Vehicle config================
static const float vehicle_width = 0.4;
static const float vehicle_length = 0.5;

/// =============Global planner config================
static const bool enable_cover_path = false;
static const std::string global_path_strategy = "hybrid_a_star";

/// =============A star config================
static const float a_star_obstacle_gap = 0.3;

/// =============Hybrid A star config================
/// [m] --- The minimum turning radius of the vehicle
static const float r = 18;
/// A flag to toggle reversing (true = on; false = off)
static const bool reverse = false;
/// A flag to toggle the connection of the path via Dubin's shot (true = on;
/// false = off)
static const bool dubinsShot = true;
/// A flag to toggle the Dubin's heuristic, this should be false, if reversing
/// is enabled (true = on; false = off)
static const bool dubins = false;

/// [#] --- Limits the maximum search depth of the algorithm, possibly
/// terminating without the solution
static const int iterations = 600000;
/// [m] --- The number of discretizations in heading
static const int headings = 72;
/// [°] --- The discretization value of the heading (goal condition)
static const float deltaHeadingDeg = 360 / (float)headings;
/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
/// [c*M_PI] --- The heading part of the goal condition
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot)
/// first triggers
static const float dubinsShotDistance = 20;
/// [#] --- A movement cost penalty for turning (choosing non straight motion
/// primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives >
/// 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from
/// primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily
/// relevant for collision checking
static const float dubinsStepSize = 1;

/// =============Local Planner config================
static const double lookaheadDistance = 0.6;
static const double maxLinearSpeed = 0.4;
static const double maxAngularSpeed = 1.0;

/// =============TEB Local Planner config================
static const double teb_dt_ref = 0.5;
static const int teb_n_points = 15;
static const double teb_wheelbase = 0.5;
static const double teb_w_obstacle = 100.0;
static const double teb_w_smoothness = 50.0;
static const double teb_w_velocity = 30.0;
static const double teb_w_time = 10.0;
static const double teb_w_kinematic = 20.0;
static const int teb_optimization_iterations = 50;
}  // namespace constants

}  // namespace common