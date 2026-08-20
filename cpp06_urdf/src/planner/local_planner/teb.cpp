#include "planner/local_planner/teb.h"

#include <g2o/core/factory.h>

#include <cmath>
#include <ctime>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "planner/local_planner/teb_types/misc.h"
#include "planner/local_planner/teb_types/pose_se2.h"

namespace local_planner {

using TebPose = teb_local_planner::PoseSE2;
using namespace teb_local_planner;

TebLocalPlanner::TebLocalPlanner(std::shared_ptr<map::StaticMap> map,
                                 const rclcpp::Logger logger)
    : LocalPlannerBase(map, logger) {
  cfg_.trajectory.dt_ref = 0.3;
  cfg_.trajectory.min_samples = 10;
  cfg_.trajectory.max_samples = 500;
  cfg_.trajectory.estimate_orient = true;
  cfg_.robot.max_vel_x = 0.4;
  cfg_.robot.max_vel_x_backwards = 0.2;
  cfg_.robot.max_vel_theta = 1.0;
  cfg_.robot.acc_lim_x = 0.5;
  cfg_.robot.acc_lim_theta = 1.0;
  cfg_.optim.max_iterations = 50;
  cfg_.optim.penalty_epsilon = 0.1;
  cfg_.obstacles.min_obstacle_dist = 0.3;

  cfg_.robot_model = std::make_shared<CircularRobotFootprintModel>(0.1);

  optimizer_ = initOptimizer();
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
}

void TebLocalPlanner::init() {
  initialized_ = false;
  teb_obstacles_.clear();
  prefer_rotdir_ = RotType::none;
}

std::shared_ptr<g2o::SparseOptimizer> TebLocalPlanner::initOptimizer() {
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances
  // (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this]() { this->registerG2OTypes(); });

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer =
      std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver =
      std::make_unique<TEBLinearSolver>();  // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  auto blockSolver = std::make_unique<TEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading();  // required for >Eigen 3.1

  return optimizer;
}

void TebLocalPlanner::registerG2OTypes() {
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType(
      "VERTEX_POSE",
      std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType(
      "VERTEX_TIMEDIFF",
      std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
  factory->registerType(
      "EDGE_TIME_OPTIMAL",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType(
      "EDGE_SHORTEST_PATH",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType(
      "EDGE_VELOCITY",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType(
      "EDGE_VELOCITY_HOLONOMIC",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType(
      "EDGE_ACCELERATION",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType(
      "EDGE_ACCELERATION_START",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType(
      "EDGE_ACCELERATION_GOAL",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC_START",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType(
      "EDGE_ACCELERATION_HOLONOMIC_GOAL",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType(
      "EDGE_KINEMATICS_DIFF_DRIVE",
      std::make_shared<
          g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType(
      "EDGE_KINEMATICS_CARLIKE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  factory->registerType(
      "EDGE_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType(
      "EDGE_INFLATED_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType(
      "EDGE_DYNAMIC_OBSTACLE",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType(
      "EDGE_VIA_POINT",
      std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType(
      "EDGE_PREFER_ROTDIR",
      std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());
  return;
}

geometry_msgs::msg::Twist TebLocalPlanner::getControlCmd() {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.angular.z = 0.0;
  bool free_goal_vel = true;

  if (smoothed_local_.poses.size() < 2) {
    RCLCPP_WARN(logger_, "TEB: No smoothed local path available");
    return cmd;
  }

  std::vector<PoseSE2> ref_path;
  for (const auto& pose_stamped : smoothed_local_.poses) {
    PoseSE2 p;
    double qx = pose_stamped.pose.orientation.x;
    double qy = pose_stamped.pose.orientation.y;
    double qz = pose_stamped.pose.orientation.z;
    double qw = pose_stamped.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    p = PoseSE2(pose_stamped.pose.position.x, pose_stamped.pose.position.y,
                std::atan2(siny_cosp, cosy_cosp));
    ref_path.push_back(p);
  }

  PoseSE2 start_pose;
  double qx = current_pose_.pose.orientation.x;
  double qy = current_pose_.pose.orientation.y;
  double qz = current_pose_.pose.orientation.z;
  double qw = current_pose_.pose.orientation.w;
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  double start_pose_theta = std::atan2(siny_cosp, cosy_cosp);

  start_pose = PoseSE2(current_pose_.pose.position.x,
                       current_pose_.pose.position.y, start_pose_theta);
  robot_pose_ = start_pose;
  PoseSE2 goal_pose = ref_path.back();
  robot_goal_ = goal_pose;

  convertMapObstacles();

  geometry_msgs::msg::Twist start_vel;
  start_vel.linear.x = current_twist_.twist.linear.x;
  start_vel.angular.z = current_twist_.twist.angular.z;
  ref_path_ = ref_path;
  plan(robot_pose_, ref_path, &start_vel, true);

  getVelocityCommand(cmd.linear.x, cmd.linear.y, cmd.angular.z);
  return cmd;
}

visualization_msgs::msg::MarkerArray TebLocalPlanner::getTrajectoryMarkers(
    const std::string& frame_id) {
  visualization_msgs::msg::MarkerArray marker_array;

  if (pose_vec_.empty()) {
    return marker_array;
  }

  // 先清空同 namespace 下的旧 marker，避免 trajectory 长度变短时残留。
  const std::string ns = "teb_trajectory";
  {
    visualization_msgs::msg::Marker delete_all;
    delete_all.header.frame_id = frame_id;
    delete_all.header.stamp = rclcpp::Clock().now();
    delete_all.ns = ns;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
  }

  // 每个 pose 一个 CUBE marker，id 递增，每次覆盖同名空间下相同 id 的 marker。
  const float box_size = 0.15f;  // 轨迹框尺寸，单位 m
  const float box_height = 0.05f;

  for (size_t i = 0; i < pose_vec_.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Pose pose;
    pose_vec_[i]->pose().toPoseMsg(pose);
    marker.pose = pose;
    marker.pose.position.z = box_height / 2.0;

    marker.scale.x = box_size;
    marker.scale.y = box_size;
    marker.scale.z = box_height;

    // 颜色随索引渐变：从绿色过渡到蓝色
    float ratio =
        pose_vec_.size() > 1
            ? static_cast<float>(i) / static_cast<float>(pose_vec_.size() - 1)
            : 0.0f;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f - ratio * 0.5f;
    marker.color.b = 0.3f + ratio * 0.7f;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

void TebLocalPlanner::setVelocityStart() {
  vel_start_.first = true;
  vel_start_.second.linear.x = current_twist_.twist.linear.x;
  vel_start_.second.linear.y = current_twist_.twist.linear.y;
  vel_start_.second.angular.z = current_twist_.twist.angular.z;
}

bool TebLocalPlanner::plan(const PoseSE2& robot_pose,
                           const std::vector<PoseSE2>& initial_plan,
                           const geometry_msgs::msg::Twist* start_vel,
                           bool free_goal_vel) {
  if (timediff_vec_.empty() || pose_vec_.empty()) {
    initTrajectory(robot_pose, initial_plan);
  } else {
    PoseSE2 goal(initial_plan.back());
    if (sizePoses() > 0 &&
        (goal.position() - BackPose().position()).norm() <
            cfg_.trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal.theta() - BackPose().theta())) <
            cfg_.trajectory.force_reinit_new_goal_angular) {
      // actual warm start!
      updateAndPruneTEB(robot_pose, goal, cfg_.trajectory.min_samples);
    } else {
      // goal too far away -> reinit
      RCLCPP_DEBUG(logger_,
                   "New goal: distance to existing goal is higher than the "
                   "specified threshold. Reinitalizing trajectories.");
      clearTimedElasticBand();
      initTrajectory(robot_pose, initial_plan);
    }
  }

  // 设置起点终点vel
  setVelocityStart();

  if (free_goal_vel) {
    vel_goal_.first = false;
  } else {
    vel_goal_.first = true;
  }

  return optimizeTEB(cfg_.optim.no_inner_iterations,
                     cfg_.optim.no_outer_iterations);
}

void TebLocalPlanner::initTrajectory(const PoseSE2& robot_pose,
                                     const std::vector<PoseSE2>& ref_points) {
  int n = static_cast<int>(ref_points.size());
  if (n < 2) {
    RCLCPP_WARN(logger_, "Number of ref_points: %d",
                (unsigned int)ref_points.size());
    return;
  }
  if (initialized_) {
    RCLCPP_WARN(logger_,
                "Cannot init TEB between given configuration and goal, because "
                "TEB vectors are not empty or TEB is already initialized (call "
                "this function before adding states yourself)!");
    RCLCPP_WARN(logger_,
                "Number of TEB configurations: %d, Number of TEB timediffs: %d",
                (unsigned int)sizePoses(), (unsigned int)sizeTimeDiffs());
    return;
  }

  PoseSE2 start(robot_pose);
  PoseSE2 goal(ref_points.back());

  // 添加teb起点
  addPose(start, true);

  double timestep = 0.1;

  // 添加中间点
  for (int i = 1; i < static_cast<int>(ref_points.size() - 1); ++i) {
    double yaw;
    if (cfg_.trajectory.estimate_orient) {
      double dx =
          ref_points[i + 1].position().x() - ref_points[i].position().x();
      double dy =
          ref_points[i + 1].position().y() - ref_points[i].position().y();
      yaw = std::atan2(dy, dx);
    } else {
      yaw = ref_points[i].theta();
    }
    PoseSE2 intermediate_pose(ref_points[i].position().x(),
                              ref_points[i].position().y(), yaw);
    double dt = estimateDeltaT(BackPose(), intermediate_pose,
                               cfg_.robot.max_vel_x, cfg_.robot.max_vel_theta);
    addPoseAndTimeDiff(intermediate_pose, dt);
  }

  if (sizePoses() < cfg_.trajectory.min_samples - 1) {
    while (sizePoses() < cfg_.trajectory.min_samples - 1) {
      PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
      double dt =
          estimateDeltaT(BackPose(), intermediate_pose, cfg_.robot.max_vel_x,
                         cfg_.robot.max_vel_theta);
      addPoseAndTimeDiff(intermediate_pose, dt);
    }
  }

  // 添加终点
  double dt = estimateDeltaT(BackPose(), goal, cfg_.robot.max_vel_x,
                             cfg_.robot.max_vel_theta);
  addPoseAndTimeDiff(goal, dt);
  setPoseVertexFixed(sizePoses() - 1, true);
}

void TebLocalPlanner::updateAndPruneTEB(const PoseSE2& new_start,
                                        std::optional<PoseSE2> new_goal,
                                        int min_samples) {
  // first and simple approach: change only start confs (and virtual start conf
  // for inital velocity) TEST if optimizer can handle this "hard" placement

  // find nearest state (using l2-norm) in order to prune the trajectory
  // (remove already passed states)
  double dist_cache = (new_start.position() - Pose(0).position()).norm();
  double dist;
  // satisfy min_samples, otherwise max 10 samples
  int lookahead = std::min<int>(sizePoses() - min_samples, 10);

  int nearest_idx = 0;
  for (int i = 1; i <= lookahead; ++i) {
    dist = (new_start.position() - Pose(i).position()).norm();
    if (dist < dist_cache) {
      dist_cache = dist;
      nearest_idx = i;
    } else {
      break;
    }
  }

  // prune trajectory at the beginning (and extrapolate sequences at the end
  // if the horizon is fixed)
  if (nearest_idx > 0) {
    // nearest_idx is equal to the number of samples to be removed (since it
    // counts from 0 ;-) ) WARNING delete starting at pose 1, and overwrite
    // the original pose(0) with new_start, since Pose(0) is fixed during
    // optimization!
    // delete first states such that the closest state is the new first one
    deletePoses(1, nearest_idx);
    // delete corresponding time differences
    deleteTimeDiffs(1, nearest_idx);
  }

  // update start
  Pose(0) = new_start;

  if (new_goal && sizePoses() > 0) {
    BackPose() = *new_goal;
  }

  // snap intermediate poses onto the current reference line
  if (ref_path_.size() >= 2 && sizePoses() > 2) {
    size_t ref_idx = 0;
    for (int i = 1; i < static_cast<int>(sizePoses()) - 1; ++i) {
      ref_idx = findNearestRefIdx(Pose(i).position(), ref_idx);
      Pose(i).position() = ref_path_.at(ref_idx).position();
      if (cfg_.trajectory.estimate_orient) {
        double dx, dy;
        if (ref_idx + 1 < ref_path_.size()) {
          dx = ref_path_.at(ref_idx + 1).x() - ref_path_.at(ref_idx).x();
          dy = ref_path_.at(ref_idx + 1).y() - ref_path_.at(ref_idx).y();
        } else {
          dx = ref_path_.at(ref_idx).x() - ref_path_.at(ref_idx - 1).x();
          dy = ref_path_.at(ref_idx).y() - ref_path_.at(ref_idx - 1).y();
        }
        Pose(i).theta() = std::atan2(dy, dx);
      } else {
        Pose(i).theta() = ref_path_.at(ref_idx).theta();
      }
    }
  }
}

size_t TebLocalPlanner::findNearestRefIdx(const Eigen::Vector2d& position,
                                          size_t hint) const {
  const size_t n = ref_path_.size();
  if (n == 0) {
    return 0;
  }
  hint = std::min(hint, n - 1);
  const size_t window = 12;
  const size_t start = hint > window ? hint - window : 0;
  const size_t end = std::min(n, hint + window + 1);
  size_t best = hint;
  double best_dist = (ref_path_.at(hint).position() - position).squaredNorm();
  for (size_t i = start; i < end; ++i) {
    double dist = (ref_path_.at(i).position() - position).squaredNorm();
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }
  return best;
}

bool TebLocalPlanner::optimizeTEB(const int iterations_innerloop,
                                  const int iterations_outerloop,
                                  bool compute_cost_afterwards,
                                  double obst_cost_scale,
                                  double viapoint_cost_scale,
                                  bool alternative_time_cost) {
  if (cfg_.optim.optimization_activate == false) {
    return false;
  }

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;
  bool fast_mode = !cfg_.obstacles.include_dynamic_obstacles;

  for (int i = 0; i < iterations_outerloop; ++i) {
    if (cfg_.trajectory.teb_autosize) {
      autoResize(fast_mode);
    }

    success = buildGraph(weight_multiplier);

    if (!success) {
      clearGraph();
      return false;
    }

    success = optimizeGraph(iterations_innerloop, false);
    if (!success) {
      clearGraph();
      return false;
    }

    optimized_ = true;
    // compute cost vec only in the last iteration
    if (compute_cost_afterwards && i == iterations_outerloop - 1) {
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale,
                         alternative_time_cost);
    }
    clearGraph();

    weight_multiplier *= cfg_.optim.weight_adapt_factor;
  }
  return true;
}

bool TebLocalPlanner::getVelocityCommand(double& vx, double& vy, double& omega,
                                         int look_ahead_poses) const {
  if (sizePoses() < 2) {
    RCLCPP_ERROR(logger_,
                 "TebOptimalPlanner::getVelocityCommand(): The trajectory "
                 "contains less than 2 poses. Make sure to init and "
                 "optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(
      1, std::min(look_ahead_poses, static_cast<int>(sizePoses() - 1)));
  double dt = 0.0;
  for (int counter = 0; counter < look_ahead_poses; ++counter) {
    dt += TimeDiff(counter);
    // TODO: change to look-ahead time? Refine trajectory?
    if (dt >= cfg_.trajectory.dt_ref * look_ahead_poses) {
      look_ahead_poses = counter + 1;
      break;
    }
  }
  if (dt <= 0) {
    RCLCPP_ERROR(
        logger_,
        "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  extractVelocity(Pose(0), Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebLocalPlanner::extractVelocity(const PoseSE2& pose1,
                                      const PoseSE2& pose2, double dt,
                                      double& vx, double& vy,
                                      double& omega) const {
  if (dt == 0) {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  if (cfg_.robot.max_vel_y == 0)  // nonholonomic robot
  {
    Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double)g2o_sign(dir) * deltaS.norm() / dt;
    vy = 0;
  } else  // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff / dt;
}

void TebLocalPlanner::autoResize(const bool fast_mode) {
  assert(sizeTimeDiffs() == 0 || sizeTimeDiffs() + 1 == sizePoses());
  /// iterate through all TEB states and add/remove states!
  bool modified = true;
  double dt_ref = cfg_.trajectory.dt_ref;
  double dt_hysteresis = cfg_.trajectory.dt_hysteresis;
  int min_samples = cfg_.trajectory.min_samples;
  int max_samples = cfg_.trajectory.max_samples;

  // actually it should be while(), but we want to make sure to not
  // get stuck in some oscillation, hence max 100 repitions.
  // 对delta_t超过范围的，通过增加和删除节点，均匀化采样时间
  for (int rep = 0; rep < 100 && modified; ++rep) {
    modified = false;

    // TimeDiff connects Point(i) with Point(i+1)
    for (int i = 0; i < sizeTimeDiffs(); ++i) {
      if (TimeDiff(i) > dt_ref + dt_hysteresis &&
          sizeTimeDiffs() < max_samples) {
        // RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"),
        // "teb_local_planner: autoResize() inserting new bandpoint i=%u,
        // #TimeDiffs=%lu",i,sizeTimeDiffs());

        double newtime = 0.5 * TimeDiff(i);

        TimeDiff(i) = newtime;
        insertPose(i + 1, PoseSE2::average(Pose(i), Pose(i + 1)));
        insertTimeDiff(i + 1, newtime);

        modified = true;
      } else if (TimeDiff(i) < dt_ref - dt_hysteresis &&
                 sizeTimeDiffs() > min_samples) {
        // only remove samples if size
        // is larger than min_samples.
        // RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"),
        // "teb_local_planner: autoResize() deleting bandpoint i=%u,
        // #TimeDiffs=%lu",i,sizeTimeDiffs());

        if (i < ((int)sizeTimeDiffs() - 1)) {
          TimeDiff(i + 1) = TimeDiff(i + 1) + TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i + 1);
        } else {
          // last motion should be adjusted, shift time to the interval
          // before
          TimeDiff(i - 1) += TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
  }
}

bool TebLocalPlanner::buildGraph(double weight_multiplier) {
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
    RCLCPP_WARN(
        logger_,
        "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  // 统计g2o的优化信息
  optimizer_->setComputeBatchStatistics(
      cfg_.recovery.divergence_detection_enable);

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  AddEdgesObstacles(weight_multiplier);

  // TODO: Add dynamic obstacle first
  // if (cfg_.obstacles.include_dynamic_obstacles) {
  //   AddEdgesDynamicObstacles();
  // }

  // TODO 对规划轨迹的中间途径点建边
  // AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  if (cfg_.robot.min_turning_radius == 0 ||
      cfg_.optim.weight_kinematics_turning_radius == 0) {
    // 全向运动模型
    AddEdgesKinematicsDiffDrive();  // we have a differential drive robot
  } else {
    // 类车运动学模型
    AddEdgesKinematicsCarlike();  // we have a carlike robot since the turning
                                  // radius is bounded from below.
  }
  // 震荡恢复，只在震荡时启动
  AddEdgesPreferRotDir();

  // 靠近障碍物时主动减速
  if (cfg_.optim.weight_velocity_obstacle_ratio > 0) {
    AddEdgesVelocityObstacleRatio();
  }

  return true;
}

void TebLocalPlanner::AddTEBVertices() {
  // add vertices to graph
  RCLCPP_DEBUG_EXPRESSION(logger_, cfg_.optim.optimization_verbose,
                          "Adding TEB vertices ...");
  unsigned int id_counter = 0;  // used for vertices ids
  obstacles_per_vertex_.resize(sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i = 0; i < sizePoses(); ++i) {
    PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(PoseVertex(i));
    if (sizeTimeDiffs() != 0 && i < sizeTimeDiffs()) {
      TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(teb_obstacles_.size());
  }
}

void TebLocalPlanner::AddEdgesObstacles(double weight_multiplier) {
  if (cfg_.optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      teb_obstacles_.empty()) {
    return;  // if weight equals zero skip adding edges!
  }

  bool inflated =
      cfg_.obstacles.inflation_dist > cfg_.obstacles.min_obstacle_dist;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_.optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_.optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_.optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this](
                         int index, const Obstacle* obstacle) {
    if (inflated) {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0, PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(cfg_, cfg_.robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    } else {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(cfg_, cfg_.robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };

  // iterate all teb points, skipping the last and, if the
  // EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex =
      cfg_.optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < sizePoses() - 1; ++i) {
    double left_min_dist = std::numeric_limits<double>::max();
    double right_min_dist = std::numeric_limits<double>::max();
    ObstaclePtr left_obstacle;
    ObstaclePtr right_obstacle;

    const Eigen::Vector2d pose_orient = Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : teb_obstacles_) {
      // we handle dynamic obstacles differently below
      if (cfg_.obstacles.include_dynamic_obstacles && obst->isDynamic())
        continue;

      // calculate distance to robot model
      double dist = cfg_.robot_model->calculateDistance(Pose(i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist <
          cfg_.obstacles.min_obstacle_dist *
              cfg_.obstacles.obstacle_association_force_inclusion_factor) {
        iter_obstacle->push_back(obst);
        continue;
      }
      // cut-off distance
      if (dist > cfg_.obstacles.min_obstacle_dist *
                     cfg_.obstacles.obstacle_association_cutoff_factor)
        continue;

      // determine side (left or right) and assign obstacle if closer than the
      // previous one
      if (cross2d(pose_orient, obst->getCentroid()) > 0)  // left
      {
        if (dist < left_min_dist) {
          left_min_dist = dist;
          left_obstacle = obst;
        }
      } else {
        if (dist < right_min_dist) {
          right_min_dist = dist;
          right_obstacle = obst;
        }
      }
    }

    if (left_obstacle) iter_obstacle->push_back(left_obstacle);
    if (right_obstacle) iter_obstacle->push_back(right_obstacle);

    // continue here to ignore obstacles for the first pose, but use them later
    // to create the EdgeVelocityObstacleRatio edges
    if (i == 0) {
      ++iter_obstacle;
      continue;
    }

    // create obstacle edges
    for (const ObstaclePtr obst : *iter_obstacle) {
      create_edge(i, obst.get());
    }
    ++iter_obstacle;
  }
}

void TebLocalPlanner::AddEdgesVelocity() {
  if (cfg_.robot.max_vel_y == 0) {
    // non-holonomic robot
    if (cfg_.optim.weight_max_vel_x == 0 &&
        cfg_.optim.weight_max_vel_theta == 0) {
      return;  // if weight equals zero skip adding edges!
    }

    int n = sizePoses();
    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = cfg_.optim.weight_max_vel_x;
    information(1, 1) = cfg_.optim.weight_max_vel_theta;
    information(0, 1) = 0.0;
    information(1, 0) = 0.0;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0, PoseVertex(i));
      velocity_edge->setVertex(1, PoseVertex(i + 1));
      velocity_edge->setVertex(2, TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  } else {
    // holonomic-robot
    if (cfg_.optim.weight_max_vel_x == 0 && cfg_.optim.weight_max_vel_y == 0 &&
        cfg_.optim.weight_max_vel_theta == 0)
      return;  // if weight equals zero skip adding edges!

    int n = sizePoses();
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_.optim.weight_max_vel_x;
    information(1, 1) = cfg_.optim.weight_max_vel_y;
    information(2, 2) = cfg_.optim.weight_max_vel_theta;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0, PoseVertex(i));
      velocity_edge->setVertex(1, PoseVertex(i + 1));
      velocity_edge->setVertex(2, TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
}

void TebLocalPlanner::AddEdgesAcceleration() {
  if (cfg_.optim.weight_acc_lim_x == 0 &&
      cfg_.optim.weight_acc_lim_theta == 0) {
    return;  // if weight equals zero skip adding edges!
  }

  int n = sizePoses();

  if (cfg_.robot.max_vel_y == 0 || cfg_.robot.acc_lim_y == 0) {
    // non-holonomic robot
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_.optim.weight_acc_lim_x;
    information(1, 1) = cfg_.optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0, PoseVertex(0));
      acceleration_edge->setVertex(1, PoseVertex(1));
      acceleration_edge->setVertex(2, TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0, PoseVertex(i));
      acceleration_edge->setVertex(1, PoseVertex(i + 1));
      acceleration_edge->setVertex(2, PoseVertex(i + 2));
      acceleration_edge->setVertex(3, TimeDiffVertex(i));
      acceleration_edge->setVertex(4, TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0, PoseVertex(n - 2));
      acceleration_edge->setVertex(1, PoseVertex(n - 1));
      acceleration_edge->setVertex(2, TimeDiffVertex(sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  } else {
    // holonomic robot
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_.optim.weight_acc_lim_x;
    information(1, 1) = cfg_.optim.weight_acc_lim_y;
    information(2, 2) = cfg_.optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      EdgeAccelerationHolonomicStart* acceleration_edge =
          new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0, PoseVertex(0));
      acceleration_edge->setVertex(1, PoseVertex(1));
      acceleration_edge->setVertex(2, TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      EdgeAccelerationHolonomic* acceleration_edge =
          new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0, PoseVertex(i));
      acceleration_edge->setVertex(1, PoseVertex(i + 1));
      acceleration_edge->setVertex(2, PoseVertex(i + 2));
      acceleration_edge->setVertex(3, TimeDiffVertex(i));
      acceleration_edge->setVertex(4, TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      EdgeAccelerationHolonomicGoal* acceleration_edge =
          new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0, PoseVertex(n - 2));
      acceleration_edge->setVertex(1, PoseVertex(n - 1));
      acceleration_edge->setVertex(2, TimeDiffVertex(sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}

void TebLocalPlanner::AddEdgesTimeOptimal() {
  if (cfg_.optim.weight_optimaltime == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_.optim.weight_optimaltime);

  for (int i = 0; i < sizeTimeDiffs(); ++i) {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebLocalPlanner::AddEdgesShortestPath() {
  if (cfg_.optim.weight_shortest_path == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_.optim.weight_shortest_path);

  for (int i = 0; i < sizePoses() - 1; ++i) {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, PoseVertex(i));
    shortest_path_edge->setVertex(1, PoseVertex(i + 1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void TebLocalPlanner::AddEdgesKinematicsDiffDrive() {
  if (cfg_.optim.weight_kinematics_nh == 0 &&
      cfg_.optim.weight_kinematics_forward_drive == 0)
    return;  // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_.optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_.optim.weight_kinematics_forward_drive;

  for (int i = 0; i < sizePoses() - 1; i++)  // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0, PoseVertex(i));
    kinematics_edge->setVertex(1, PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebLocalPlanner::AddEdgesKinematicsCarlike() {
  if (cfg_.optim.weight_kinematics_nh == 0 &&
      cfg_.optim.weight_kinematics_turning_radius == 0)
    return;  // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_.optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_.optim.weight_kinematics_turning_radius;

  for (int i = 0; i < sizePoses() - 1; i++)  // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0, PoseVertex(i));
    kinematics_edge->setVertex(1, PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebLocalPlanner::AddEdgesPreferRotDir() {
  // TODO(roesmann): Note, these edges can result in odd predictions, in
  // particular
  //                 we can observe a substantional mismatch between open- and
  //                 closed-loop planning leading to a poor control performance.
  //                 At the moment, we keep these functionality for oscillation
  //                 recovery: Activating the edge for a short time period might
  //                 not be crucial and could move the robot to a new
  //                 oscillation-free state. This needs to be analyzed in more
  //                 detail!
  if (prefer_rotdir_ == RotType::none || cfg_.optim.weight_prefer_rotdir == 0)
    return;  // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left) {
    RCLCPP_WARN(logger_,
                "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported "
                "RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 1, 1> information_rotdir;
  information_rotdir.fill(cfg_.optim.weight_prefer_rotdir);
  // currently: apply to first 3 rotations
  for (int i = 0; i < sizePoses() - 1 && i < 3; ++i) {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0, PoseVertex(i));
    rotdir_edge->setVertex(1, PoseVertex(i + 1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left) {
      rotdir_edge->preferLeft();
    } else if (prefer_rotdir_ == RotType::right) {
      rotdir_edge->preferRight();
    }

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebLocalPlanner::AddEdgesVelocityObstacleRatio() {
  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_.optim.weight_velocity_obstacle_ratio;
  information(1, 1) = cfg_.optim.weight_velocity_obstacle_ratio;
  information(0, 1) = information(1, 0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < sizePoses() - 1; ++index) {
    for (const ObstaclePtr obstacle : (*iter_obstacle++)) {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0, PoseVertex(index));
      edge->setVertex(1, PoseVertex(index + 1));
      edge->setVertex(2, TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(cfg_, cfg_.robot_model.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebLocalPlanner::optimizeGraph(int no_iterations, bool clear_after) {
  if (cfg_.robot.max_vel_x < 0.01) {
    RCLCPP_WARN(logger_,
                "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. "
                "Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;
  }

  if (!isInit() || sizePoses() < cfg_.trajectory.min_samples) {
    RCLCPP_WARN(logger_,
                "optimizeGraph(): TEB is empty or has too less elements. "
                "Skipping optimization.");
    if (clear_after) clearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_.optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if (!iter) {
    RCLCPP_ERROR(logger_, "optimizeGraph(): Optimization failed! iter=%i",
                 iter);
    return false;
  }

  if (clear_after) clearGraph();

  return true;
}

void TebLocalPlanner::clearGraph() {  // clear optimizer states
  if (optimizer_) {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for (auto& v : vertices) v.second->edges().clear();

    optimizer_->vertices()
        .clear();  // necessary, because optimizer->clear deletes
                   // pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}

void TebLocalPlanner::computeCurrentCost(double obst_cost_scale,
                                         double viapoint_cost_scale,
                                         bool alternative_time_cost) {
  // check if graph is empty/exist  -> important if function is called between
  // buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
    // here the graph is build again, for time efficiency make sure to call this
    // function between buildGraph and Optimize (deleted), but it depends on the
    // application
    buildGraph(1.0);
    optimizer_->initializeOptimization();
  } else {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost) {
    cost_ += getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number
    // of samples, which is not always the same for similar TEBs, since we are
    // using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it =
           optimizer_->activeEdges().begin();
       it != optimizer_->activeEdges().end(); it++) {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr) {
      cur_cost *= obst_cost_scale;
    } else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr) {
      cur_cost *= viapoint_cost_scale;
    } else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr &&
               alternative_time_cost) {
      continue;  // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) clearGraph();
}

double TebLocalPlanner::getSumOfAllTimeDiffs() const {
  double time = 0;

  for (TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin();
       dt_it != timediff_vec_.end(); ++dt_it) {
    time += (*dt_it)->dt();
  }
  return time;
}

void TebLocalPlanner::convertMapObstacles() {
  teb_obstacles_.clear();
  if (!map_) return;

  auto poly_obstacles = map_->getObstaclesWorld();
  for (const auto& poly : poly_obstacles) {
    if (poly.size() >= 3) {
      teb_obstacles_.push_back(
          std::make_shared<teb_local_planner::PolygonObstacle>(poly));
    } else if (poly.size() == 1) {
      teb_obstacles_.push_back(
          std::make_shared<teb_local_planner::PointObstacle>(poly[0].x,
                                                             poly[0].y));
    }
  }
}

void TebLocalPlanner::addTimeDiff(double dt, bool fixed) {
  assert(dt > 0.0 && "Adding a timediff requires a positive dt");
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
  timediff_vec_.push_back(timediff_vertex);
  return;
}

void TebLocalPlanner::addPoseAndTimeDiff(const PoseSE2& pose, double dt) {
  if (sizePoses() != sizeTimeDiffs()) {
    addPose(pose, false);
    addTimeDiff(dt, false);
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("teb_local_planner"),
        "Method addPoseAndTimeDiff: Add one single Pose first. Timediff "
        "describes the time difference between last conf and given conf");
  }
  return;
}

void TebLocalPlanner::setPoseVertexFixed(int index, bool status) {
  assert(index < sizePoses());
  pose_vec_.at(index)->setFixed(status);
}

void TebLocalPlanner::addPose(const PoseSE2& pose, bool fixed) {
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

void TebLocalPlanner::addPose(double x, double y, double theta, bool fixed) {
  VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
  pose_vec_.push_back(pose_vertex);
  return;
}

double TebLocalPlanner::estimateDeltaT(const PoseSE2& start, const PoseSE2& end,
                                       double max_vel_x, double max_vel_theta) {
  double dt_constant_motion = 0.1;
  if (max_vel_x > 0) {
    double trans_dist = (end.position() - start.position()).norm();
    dt_constant_motion = trans_dist / max_vel_x;
  }
  if (max_vel_theta > 0) {
    double rot_dist =
        std::abs(g2o::normalize_theta(end.theta() - start.theta()));
    dt_constant_motion = std::max(dt_constant_motion, rot_dist / max_vel_theta);
  }
  return dt_constant_motion;
}

void TebLocalPlanner::clearTimedElasticBand() {
  for (PoseSequence::iterator pose_it = pose_vec_.begin();
       pose_it != pose_vec_.end(); ++pose_it)
    delete *pose_it;
  pose_vec_.clear();

  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin();
       dt_it != timediff_vec_.end(); ++dt_it)
    delete *dt_it;
  timediff_vec_.clear();
}

void TebLocalPlanner::deletePoses(int index, int number) {
  assert(index + number <= (int)pose_vec_.size());
  for (int i = index; i < index + number; ++i) delete pose_vec_.at(i);
  pose_vec_.erase(pose_vec_.begin() + index,
                  pose_vec_.begin() + index + number);
}

void TebLocalPlanner::deleteTimeDiffs(int index, int number) {
  assert(index + number <= timediff_vec_.size());
  for (int i = index; i < index + number; ++i) delete timediff_vec_.at(i);
  timediff_vec_.erase(timediff_vec_.begin() + index,
                      timediff_vec_.begin() + index + number);
}

}  // namespace local_planner
