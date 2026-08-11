#pragma once

namespace teb_local_planner {

struct TebConfig {
  struct Robot {
    double max_vel_x = 0.4;
    double max_vel_x_backwards = 0.2;
    double max_vel_y = 0.0;
    double max_vel_theta = 1.0;
    double acc_lim_x = 0.5;
    double acc_lim_y = 0.0;
    double acc_lim_theta = 1.0;
    double min_turning_radius = 0.0;
    double wheelbase = 0.5;
  } robot;

  struct Trajectory {
    double dt_ref = 0.3;
    int min_samples = 3;
    int max_samples = 500;
    bool exact_arc_length = false;
    bool estimate_orient = true;
    double dt_hysteresis = 0.1;
  } trajectory;

  struct Optim {
    int max_iterations = 50;
    double penalty_epsilon = 0.1;
    double obstacle_cost_exponent = 1.0;
  } optim;

  struct Obstacles {
    double min_obstacle_dist = 0.3;
    double inflation_dist = 0.6;
  } obstacles;

  struct Weights {
    double weight_obstacle = 100.0;
    double weight_inflation = 10.0;
    double weight_velocity = 30.0;
    double weight_acceleration = 10.0;
    double weight_kinematics_nh = 1000.0;
    double weight_kinematics_forward_drive = 1.0;
    double weight_shortest_path = 50.0;
    double weight_time_optimal = 10.0;
    double weight_via_point = 100.0;
    double weight_prefer_rotdir = 10.0;
  } weights;
};

}  // namespace teb_local_planner
