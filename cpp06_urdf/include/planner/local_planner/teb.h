#pragma once

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "map/static_map.h"
#include "planner/local_planner/local_planner_base.h"
#include "planner/local_planner/teb_types/edge_acceleration.h"
#include "planner/local_planner/teb_types/edge_dynamic_obstacle.h"
#include "planner/local_planner/teb_types/edge_kinematics.h"
#include "planner/local_planner/teb_types/edge_obstacle.h"
#include "planner/local_planner/teb_types/edge_prefer_rotdir.h"
#include "planner/local_planner/teb_types/edge_shortest_path.h"
#include "planner/local_planner/teb_types/edge_time_optimal.h"
#include "planner/local_planner/teb_types/edge_velocity.h"
#include "planner/local_planner/teb_types/edge_velocity_obstacle_ratio.h"
#include "planner/local_planner/teb_types/edge_via_point.h"
#include "planner/local_planner/teb_types/obstacles.h"
#include "planner/local_planner/teb_types/robot_footprint_model.h"
#include "planner/local_planner/teb_types/teb_config.h"
#include "planner/local_planner/teb_types/vertex_pose.h"
#include "planner/local_planner/teb_types/vertex_timediff.h"

namespace local_planner {

using namespace teb_local_planner;
typedef std::vector<VertexPose*> PoseSequence;
typedef std::vector<VertexTimeDiff*> TimeDiffSequence;
typedef g2o::BlockSolverX TEBBlockSolver;
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType>
    TEBLinearSolver;

class TebLocalPlanner : public LocalPlannerBase {
 public:
  TebLocalPlanner(std::shared_ptr<map::StaticMap> map,
                  const rclcpp::Logger logger);

  void init() override;

  geometry_msgs::msg::Twist getControlCmd() override;

  visualization_msgs::msg::MarkerArray getTrajectoryMarkers(
      const std::string& frame_id = "map") override;

  // std::vector<Pose2D> getOptimizedTrajectory() const;

  void setEstimateOrient(bool flag) { cfg_.trajectory.estimate_orient = flag; }

 private:
  bool isInitialized() const { return initialized_; }

  std::shared_ptr<g2o::SparseOptimizer> initOptimizer();

  static void registerG2OTypes();

  size_t sizePoses() const { return pose_vec_.size(); }

  size_t sizeTimeDiffs() const { return timediff_vec_.size(); }

  bool plan(const PoseSE2& robot_pose,
            const std::vector<PoseSE2>& initial_plan,
            const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel);

  bool getVelocityCommand(double& vx, double& vy, double& omega,
                          int look_ahead_poses = 1) const;

  void extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt,
                       double& vx, double& vy, double& omega) const;

  void initTrajectory(const PoseSE2& robot_pose,
                      const std::vector<PoseSE2>& ref_points);

  void updateAndPruneTEB(const PoseSE2& new_start,
                         std::optional<PoseSE2> new_goal,
                         int min_samples);

  size_t findNearestRefIdx(const Eigen::Vector2d& position,
                           size_t hint) const;

  // Pose function
  PoseSE2& Pose(int index) {
    assert(index < sizePoses());
    return pose_vec_.at(index)->pose();
  }

  const PoseSE2& Pose(int index) const {
    assert(index < sizePoses());
    return pose_vec_.at(index)->pose();
  }

  void addPose(double x, double y, double theta, bool fixed = false);

  void addPose(const PoseSE2& pose, bool fixed = false);

  void insertPose(int index, const PoseSE2& pose) {
    VertexPose* pose_vertex = new VertexPose(pose);
    pose_vec_.insert(pose_vec_.begin() + index, pose_vertex);
  }

  void deletePose(int index) {
    assert(index < pose_vec_.size());
    delete pose_vec_.at(index);
    pose_vec_.erase(pose_vec_.begin() + index);
  }

  void deletePoses(int index, int number);

  VertexPose* PoseVertex(int index) {
    assert(index < sizePoses());
    return pose_vec_.at(index);
  }

  PoseSE2& BackPose() { return pose_vec_.back()->pose(); }

  void setPoseVertexFixed(int index, bool status);

  // Timediff function
  double& TimeDiff(int index) {
    assert(index < sizeTimeDiffs());
    return timediff_vec_.at(index)->dt();
  }

  const double& TimeDiff(int index) const {
    assert(index < sizeTimeDiffs());
    return timediff_vec_.at(index)->dt();
  }

  void addTimeDiff(double dt, bool fixed = false);

  void insertTimeDiff(int index, double dt) {
    VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
    timediff_vec_.insert(timediff_vec_.begin() + index, timediff_vertex);
  }

  VertexTimeDiff* TimeDiffVertex(int index) {
    assert(index < sizeTimeDiffs());
    return timediff_vec_.at(index);
  }

  void deleteTimeDiff(int index) {
    assert(index < (int)timediff_vec_.size());
    delete timediff_vec_.at(index);
    timediff_vec_.erase(timediff_vec_.begin() + index);
  }

  void deleteTimeDiffs(int index, int number);

  void addPoseAndTimeDiff(const PoseSE2& pose, double dt);

  void setVelocityStart();

  void autoResize();

  void buildGraph();

  bool optimize();

  bool optimizeTEB(const int iterations_innerloop,
                   const int iterations_outerloop,
                   bool compute_cost_afterwards = false,
                   double obst_cost_scale = 1.0,
                   double viapoint_cost_scale = 1.0,
                   bool alternative_time_cost = false);

  void autoResize(const bool fast_mode);

  bool buildGraph(double weight_multiplier);

  void AddTEBVertices();

  void AddEdgesObstacles(double weight_multiplier);

  void AddEdgesVelocity();

  void AddEdgesAcceleration();

  void AddEdgesTimeOptimal();

  void AddEdgesShortestPath();

  void AddEdgesKinematicsDiffDrive();

  void AddEdgesKinematicsCarlike();

  void AddEdgesPreferRotDir();

  void AddEdgesVelocityObstacleRatio();

  bool optimizeGraph(int no_iterations, bool clear_after);

  void computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale,
                          bool alternative_time_cost);

  bool isInit() const { return !timediff_vec_.empty() && !pose_vec_.empty(); }

  bool extractVelocity(double& v, double& omega);

  void clearGraph();

  void pruneTrajectory(const PoseSE2& current_pose);

  double distance(const PoseSE2& a, const PoseSE2& b) const;

  void convertMapObstacles();

  double estimateDeltaT(const PoseSE2& start, const PoseSE2& end,
                        double max_vel_x, double max_vel_theta);

  void clearTimedElasticBand();

  TebConfig cfg_;

  std::shared_ptr<g2o::SparseOptimizer> optimizer_;
  bool optimized_ = false;

  PoseSE2 robot_pose_;  //!< Store current robot pose
  PoseSE2 robot_goal_;

  std::vector<PoseSE2> ref_path_;  //!< Store current reference line

  std::vector<VertexPose*> pose_vec_;
  std::vector<VertexTimeDiff*> timediff_vec_;

  std::vector<std::vector<std::shared_ptr<Obstacle>>> obstacles_per_vertex_;

  bool initialized_ = false;
  // Pose2D last_goal_pose_;

  std::vector<std::shared_ptr<Obstacle>> teb_obstacles_;
  std::shared_ptr<CircularRobotFootprintModel> robot_model_;
  //!< Store the initial velocity at the start pose
  std::pair<bool, geometry_msgs::msg::Twist> vel_start_;
  //!< Store the final velocity at the goal pose
  std::pair<bool, geometry_msgs::msg::Twist> vel_goal_;
  RotType prefer_rotdir_;
  //!< Store cost value of the current hyper-graph
  double cost_;

  double getSumOfAllTimeDiffs() const;

  geometry_msgs::msg::Twist last_cmd_;
};

}  // namespace local_planner
