#pragma once

namespace common {

namespace constants {

static const float vehicle_width = 0.4;
static const float vehicle_length = 0.5;

// Hybrid A star config
/// [m] --- The minimum turning radius of the vehicle
static const float r = 6;
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
static const int iterations = 100000;
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
static const float dubinsShotDistance = 100;
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
}  // namespace constants

}  // namespace common