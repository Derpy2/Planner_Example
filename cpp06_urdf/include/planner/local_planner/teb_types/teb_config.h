#pragma once

#include "robot_footprint_model.h"

namespace teb_local_planner {

struct TebConfig {
 public:
  std::string odom_topic;  //!< Topic name of the odometry message, provided by
                           //!< the robot driver or simulator
  std::string map_frame;   //!< Global planning frame
  std::string node_name;   //!< node name used for parameter event callback

  RobotFootprintModelPtr robot_model;
  std::string model_name;
  double radius;
  std::vector<double> line_start, line_end;
  double front_offset, front_radius, rear_offset, rear_radius;
  std::string footprint_string;

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
    double reinit_new_goal_dist = 1.0;
    double force_reinit_new_goal_angular = 0.5 * M_PI;
    bool teb_autosize = true;
    double force_reinit_new_goal_dist = 1;
    bool global_plan_overwrite_orientation = true;
    bool allow_init_with_backwards_motion = false;
    int control_look_ahead_poses = 1;
  } trajectory;

  struct Optim {
    int max_iterations = 50;
    double penalty_epsilon = 0.1;
    double obstacle_cost_exponent = 1.0;
    int weight_obstacle = 50;
    bool optimization_activate = true;
    double weight_adapt_factor = 2.0;
    bool optimization_verbose = false;
    double weight_inflation = 0.1;
    double weight_velocity_obstacle_ratio = 0;
    double weight_kinematics_turning_radius = 1;
    double weight_max_vel_x = 2;
    double weight_max_vel_y = 2;
    double weight_max_vel_theta = 1;
    double weight_acc_lim_x = 1;
    double weight_acc_lim_y = 1;
    double weight_acc_lim_theta = 1;
    double weight_optimaltime = 1;
    double weight_shortest_path = 0;
    double weight_kinematics_nh = 1000;
    double weight_kinematics_forward_drive = 1;
    double weight_prefer_rotdir = 50;
    int no_inner_iterations = 5;
    int no_outer_iterations = 4;
  } optim;

  struct Obstacles {
    double min_obstacle_dist = 0.3;
    double inflation_dist = 0.6;
    bool include_dynamic_obstacles = false;
    double obstacle_association_force_inclusion_factor = 1.5;
    double obstacle_association_cutoff_factor = 5;
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

  struct Recovery {
    bool divergence_detection_enable = false;
  } recovery;
};

}  // namespace teb_local_planner
