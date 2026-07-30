#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "common/geometry.h"
#include "common/node3d.h"
#include "planner/local_planner/local_planner_base.h"
#include "visualization/visualization_manager.h"

namespace local_planner {

using namespace common;

namespace {
constexpr double eplison = 1e-6;
}

struct DWACost {
  double heading_cost;
  double vel_cost;
  double obstacle_cost;
  double dist_cost;
  double toal_cost;
};

struct TrajectorySample {
  std::vector<Node3D> nodes;
  double v;
  double omega;
  DWACost cost;
  bool is_best;
};

class LocalPlannerDWA : public LocalPlannerBase {
 public:
  LocalPlannerDWA(std::shared_ptr<map::StaticMap> map,
                  const rclcpp::Logger logger)
      : LocalPlannerBase(map, logger) {}

  void init() override;

  Node3D motionModel(const Node3D& start, const double v, const double omega);

  std::vector<Node3D> predictTrajectory(const double v, const double omega,
                                        const Node3D start_node);

  bool isSafe(const std::vector<Node3D>& traj, const double v,
              const double omega, double& min_d);

  DWACost computeCost(const std::vector<Node3D>& traj, double v,
                      const Node3D& goal, const double min_dist);

  geometry_msgs::msg::Twist getControlCmd() override;

  geometry_msgs::msg::PoseStamped getNearGoal();

  std::vector<TrajectorySample> getSampledTrajectories() const {
    return sampled_trajectories_;
  }

  void clearSampledTrajectories() { sampled_trajectories_.clear(); }

  void visualizeSampledTrajectories(const std::string& frame_id = "map");

 private:
  double max_v_ = 0.6;
  double min_v_ = -0.6;
  double max_omega_ = 1.0;
  double min_omega_ = -1.0;
  double max_acc_v_ = 5.0;
  double max_decl_v_ = 5.0;
  double max_acc_omega_ = 1.0;
  double max_decl_omega_ = 1.0;

  double v_resolution_ = 0.05;
  double omega_resolution_ = 0.01;

  double dt_ = 0.05;
  double total_time_ = 2.0;
  int sim_step_;

  double safe_radius = 8.0;
  double lookahead_dist_ = 10.0;

  double weight_heading_ = 0.5;
  double weight_velocity_ = 0.5;
  double weight_obstacle_ = 1.0;
  double weight_dist_ = 8.0;

  std::vector<TrajectorySample> sampled_trajectories_;
};

}  // namespace local_planner