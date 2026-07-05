#include "planner/global_planner/rrt_star.h"

namespace global_planner {

RRTStar::RRTStar(std::shared_ptr<map::StaticMap> map,
                 const rclcpp::Logger& logger)
    : GlobalPlannerBase(map, logger), collision_detector_(map) {
  double min_x = 0.0, max_x = map_->width();
  double min_y = 0.0, max_y = map_->height();
  dis_x_ = std::uniform_real_distribution<double>(min_x, max_x);
  dis_y_ = std::uniform_real_distribution<double>(min_y, max_y);
  goal_bias_ = std::uniform_real_distribution<double>(0.0, 1.0);
}

nav_msgs::msg::Path RRTStar::searchPath() {
  std::vector<std::shared_ptr<TreeNode>> tree;

  std::shared_ptr<TreeNode> start_tree_node =
      std::make_shared<TreeNode>(start_node_, std::weak_ptr<TreeNode>());
  tree.push_back(start_tree_node);

  std::shared_ptr<TreeNode> best_goal_node = nullptr;

  for (int i = 0; i < iteration_; ++i) {
    // 1. 生成地图上的随机点
    const std::shared_ptr<TreeNode> random_node = generateRandomNode();
    const std::shared_ptr<TreeNode> nearest =
        findNearestNode(tree, random_node);
    const std::shared_ptr<TreeNode> new_node =
        generateNewNode(nearest, random_node);

    if (!collision_detector_.isTraversable(new_node->node.get())) {
      continue;
    }

    if (isLineCollision(nearest->node, new_node->node)) {
      continue;
    }

    auto parent_node = chooseParent(new_node, tree);
    new_node->parent = parent_node;
    new_node->node->setG(parent_node->node->getG() +
                         calculateDistance(parent_node->node, new_node->node));

    rewire(new_node, tree);
    tree.push_back(new_node);

    if (isGoalReached(new_node)) {
      best_goal_node = new_node;
      break;
    }
  }

  if (!best_goal_node) {
    for (const auto& node : tree) {
      if (isGoalReached(node)) {
        if (!best_goal_node ||
            node->node->getG() < best_goal_node->node->getG()) {
          best_goal_node = node;
        }
      }
    }
  }

  if (!best_goal_node) {
    RCLCPP_WARN(logger_, "RRT*: Failed to find a path");
    return nav_msgs::msg::Path();
  }

  const std::vector<std::shared_ptr<TreeNode>> path_nodes =
      getPath(best_goal_node);
  return constructPathMsg(path_nodes);
}

std::shared_ptr<TreeNode> RRTStar::generateRandomNode() {
  if (goal_bias_(gen_) < goal_bias_rate_) {
    return std::make_shared<TreeNode>(
        std::make_shared<Node3D>(goal_node_->getX(), goal_node_->getY(), 0, 0,
                                 0, nullptr),
        std::weak_ptr<TreeNode>());
  }

  double x = dis_x_(gen_);
  double y = dis_y_(gen_);
  return std::make_shared<TreeNode>(
      std::make_shared<Node3D>(x, y, 0, 0, 0, nullptr),
      std::weak_ptr<TreeNode>());
}

std::shared_ptr<TreeNode> RRTStar::findNearestNode(
    const std::vector<std::shared_ptr<TreeNode>>& tree,
    const std::shared_ptr<TreeNode>& random_node) {
  double min_dist = std::numeric_limits<double>::max();
  std::shared_ptr<TreeNode> nearest = nullptr;

  for (const auto& node : tree) {
    double dist = calculateDistance(node->node, random_node->node);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = node;
    }
  }
  return nearest;
}

std::shared_ptr<TreeNode> RRTStar::generateNewNode(
    const std::shared_ptr<TreeNode>& nearest,
    const std::shared_ptr<TreeNode>& random_node) {
  double dx = random_node->node->getX() - nearest->node->getX();
  double dy = random_node->node->getY() - nearest->node->getY();
  double dist = std::sqrt(dx * dx + dy * dy);

  double extend_length = std::min(extend_length_, dist);

  double theta = std::atan2(dy, dx);
  double new_x = nearest->node->getX() + extend_length * std::cos(theta);
  double new_y = nearest->node->getY() + extend_length * std::sin(theta);

  return std::make_shared<TreeNode>(
      std::make_shared<Node3D>(new_x, new_y, theta, 0, 0, nullptr), nearest);
}

std::shared_ptr<TreeNode> RRTStar::chooseParent(
    const std::shared_ptr<TreeNode>& new_node,
    const std::vector<std::shared_ptr<TreeNode>>& tree) {
  double min_cost = std::numeric_limits<double>::max();
  std::shared_ptr<TreeNode> best_parent = nullptr;

  for (const auto& node : tree) {
    double dist = calculateDistance(node->node, new_node->node);
    if (dist > search_radius_) {
      continue;
    }

    if (isLineCollision(node->node, new_node->node)) {
      continue;
    }

    double cost = node->node->getG() + dist;
    if (cost < min_cost) {
      min_cost = cost;
      best_parent = node;
    }
  }

  return best_parent ? best_parent : tree.front();
}

void RRTStar::rewire(std::shared_ptr<TreeNode> new_node,
                     std::vector<std::shared_ptr<TreeNode>>& tree) {
  for (auto& node : tree) {
    if (node == new_node) {
      continue;
    }

    double dist = calculateDistance(new_node->node, node->node);
    if (dist > search_radius_) {
      continue;
    }

    if (isLineCollision(new_node->node, node->node)) {
      continue;
    }

    double new_cost = new_node->node->getG() + dist;
    if (new_cost < node->node->getG()) {
      node->parent = new_node;
      node->node->setG(new_cost);
    }
  }
}

bool RRTStar::isGoalReached(const std::shared_ptr<TreeNode>& node) {
  double dist = calculateDistance(node->node, goal_node_);
  return dist < goal_tolerance_;
}

std::vector<std::shared_ptr<TreeNode>> RRTStar::getPath(
    const std::shared_ptr<TreeNode>& goal_node) {
  std::vector<std::shared_ptr<TreeNode>> path;
  std::shared_ptr<TreeNode> current = goal_node;

  while (current) {
    path.push_back(current);
    auto parent = current->parent.lock();
    current = parent;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

nav_msgs::msg::Path RRTStar::constructPathMsg(
    const std::vector<std::shared_ptr<TreeNode>>& path_nodes) {
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = rclcpp::Clock().now();
  path_msg.header.frame_id = "map";
  double wx, wy;
  for (const auto& t_node : path_nodes) {
    geometry_msgs::msg::PoseStamped pose;
    map_->gridToWorld(t_node->node->getX(), t_node->node->getY(), wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  return path_msg;
}

double RRTStar::calculateDistance(const std::shared_ptr<Node3D>& a,
                                  const std::shared_ptr<Node3D>& b) {
  double dx = a->getX() - b->getX();
  double dy = a->getY() - b->getY();
  return std::sqrt(dx * dx + dy * dy);
}

bool RRTStar::isLineCollision(const std::shared_ptr<Node3D>& start,
                              const std::shared_ptr<Node3D>& end) {
  double dist = calculateDistance(start, end);
  // grid map 单位长度为1.0
  int steps = std::ceil(dist);
  steps = std::max(steps, 1);

  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double x = start->getX() + t * (end->getX() - start->getX());
    double y = start->getY() + t * (end->getY() - start->getY());

    auto check_node = std::make_shared<Node3D>(x, y, 0, 0, 0, nullptr);
    if (!collision_detector_.isTraversable(check_node.get())) {
      return true;
    }
  }
  return false;
}

}  // namespace global_planner