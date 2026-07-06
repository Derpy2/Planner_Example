#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <random>
#include <vector>

#include "common/collision_detection.h"
#include "common/node3d.h"
#include "planner/global_planner/global_planner_base.h"

namespace global_planner {

using namespace common;

struct TreeNode {
  std::shared_ptr<Node3D> node;
  std::weak_ptr<TreeNode> parent;

  TreeNode(std::shared_ptr<Node3D> n, std::weak_ptr<TreeNode> p)
      : node(n), parent(p) {}
};

class RRTStar : public GlobalPlannerBase {
 public:
  RRTStar(std::shared_ptr<map::StaticMap> map, const rclcpp::Logger& logger);

  nav_msgs::msg::Path searchPath() override;

  void setStartPose(const geometry_msgs::msg::Pose& start_pose) override;

  void setGoalPose(const geometry_msgs::msg::Pose& goal_pose) override;

  void setIteration(int iteration) { iteration_ = iteration; }
  void setExtendLength(double extend_length) { extend_length_ = extend_length; }
  void setSearchRadius(double search_radius) { search_radius_ = search_radius; }
  void setGoalTolerance(double tolerance) { goal_tolerance_ = tolerance; }

 private:
  std::shared_ptr<TreeNode> generateRandomNode();

  std::shared_ptr<TreeNode> findNearestNode(
      const std::vector<std::shared_ptr<TreeNode>>& tree,
      const std::shared_ptr<TreeNode>& random_node);

  std::shared_ptr<TreeNode> generateNewNode(
      const std::shared_ptr<TreeNode>& nearest,
      const std::shared_ptr<TreeNode>& random_node);

  std::shared_ptr<TreeNode> chooseParent(
      const std::shared_ptr<TreeNode>& new_node,
      const std::vector<std::shared_ptr<TreeNode>>& tree);

  void rewire(std::shared_ptr<TreeNode> new_node,
              std::vector<std::shared_ptr<TreeNode>>& tree);

  bool isGoalReached(const std::shared_ptr<TreeNode>& node);

  std::vector<std::shared_ptr<TreeNode>> getPath(
      const std::shared_ptr<TreeNode>& goal_node);

  nav_msgs::msg::Path constructPathMsg(
      const std::vector<std::shared_ptr<TreeNode>>& path_nodes);

  double calculateDistance(const std::shared_ptr<Node3D>& a,
                           const std::shared_ptr<Node3D>& b);

  bool isLineCollision(const std::shared_ptr<Node3D>& start,
                       const std::shared_ptr<Node3D>& end);

  int iteration_ = 5000;
  double extend_length_ = 10.0;
  double search_radius_ = 50.0;
  double goal_tolerance_ = 5.0;
  double goal_bias_rate_ = 0.05;

  std::shared_ptr<Node3D> start_node_;
  std::shared_ptr<Node3D> goal_node_;
  common::CollisionDetection collision_detector_;
  std::mt19937 gen_{std::random_device{}()};
  std::uniform_real_distribution<double> dis_x_;
  std::uniform_real_distribution<double> dis_y_;
  std::uniform_real_distribution<double> goal_bias_;
};

}  // namespace global_planner
