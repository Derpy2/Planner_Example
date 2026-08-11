#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "map/static_map.h"
#include "planner/local_planner/local_planner_base.h"
#include "planner/local_planner/teb_types/edge_acceleration.h"
#include "planner/local_planner/teb_types/edge_kinematics.h"
#include "planner/local_planner/teb_types/edge_obstacle.h"
#include "planner/local_planner/teb_types/edge_shortest_path.h"
#include "planner/local_planner/teb_types/edge_time_optimal.h"
#include "planner/local_planner/teb_types/edge_velocity.h"
#include "planner/local_planner/teb_types/edge_via_point.h"
#include "planner/local_planner/teb_types/obstacles.h"
#include "planner/local_planner/teb_types/robot_footprint_model.h"
#include "planner/local_planner/teb_types/teb_config.h"
#include "planner/local_planner/teb_types/vertex_pose.h"
#include "planner/local_planner/teb_types/vertex_timediff.h"

namespace local_planner {

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner(std::shared_ptr<map::StaticMap> map,
                  const rclcpp::Logger logger);

  void init() override;

  geometry_msgs::msg::Twist getControlCmd() override;

  std::vector<Pose2D> getOptimizedTrajectory() const;

  void setEstimateOrient(bool flag) { cfg_.trajectory.estimate_orient = flag; }

 private:
  bool isInitialized() const { return initialized_; }

  void initTrajectory(const std::vector<Pose2D>& ref_points);

  void hotStart(const std::vector<Pose2D>& ref_points,
                const Pose2D& current_pose, const Pose2D& goal_pose);

  void autoResize();

  void buildGraph();

  bool optimize();

  bool extractVelocity(double& v, double& omega);

  void clearGraph();

  void pruneTrajectory(const Pose2D& current_pose);

  double distance(const Pose2D& a, const Pose2D& b) const;

  void convertMapObstacles();

  teb_local_planner::TebConfig cfg_;

  std::unique_ptr<g2o::SparseOptimizer> optimizer_;

  std::vector<teb_local_planner::VertexPose*> pose_vec_;
  std::vector<teb_local_planner::VertexTimeDiff*> timediff_vec_;

  std::vector<Pose2D> teb_poses_;
  std::vector<double> teb_timediffs_;

  std::vector<Pose2D> prev_teb_poses_;
  std::vector<double> prev_teb_timediffs_;
  bool initialized_ = false;
  Pose2D last_goal_pose_;

  std::vector<std::shared_ptr<teb_local_planner::Obstacle>> teb_obstacles_;
  std::shared_ptr<teb_local_planner::CircularRobotFootprintModel> robot_model_;

  geometry_msgs::msg::Twist last_cmd_;
};

}  // namespace local_planner
