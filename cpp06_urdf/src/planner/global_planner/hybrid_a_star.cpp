#include "planner/global_planner/hybrid_a_star.h"

#include <cmath>
#include <queue>
#include <vector>

#include "common/config.h"
#include "common/util.h"

namespace global_planner {
using namespace common;
namespace {
constexpr double epsilon = 1e-7;
struct CompareNode3D {
  bool operator()(const std::shared_ptr<Node3D> lhs,
                  const std::shared_ptr<Node3D> rhs) const {
    return lhs->getC() > rhs->getC();
  }
};
}  // namespace

nav_msgs::msg::Path HybridAStar::searchPath() {
  int gx, gy;
  double roll, pitch, yaw;
  tf2::Quaternion q_tf;
  tf2::fromMsg(start_pose_.orientation, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  if (yaw < 0) yaw += 2 * M_PI;
  map_->worldToGrid(start_pose_.position.x, start_pose_.position.y, gx, gy);
  Node3D start = Node3D(gx, gy, yaw, 0, 0, nullptr);
  tf2::fromMsg(goal_pose_.orientation, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  if (yaw < 0) yaw += 2 * M_PI;
  map_->worldToGrid(goal_pose_.position.x, goal_pose_.position.y, gx, gy);
  Node3D goal = Node3D(gx, gy, yaw, 0, 0, nullptr);
  RCLCPP_INFO(logger_, "Hybrid a star: start pose: %f %f end pose: %f %f\n",
              start_pose_.position.x, start_pose_.position.y,
              goal_pose_.position.x, goal_pose_.position.y);
  RCLCPP_INFO(logger_, "Hybrid a star: start %f %f end %f %f \n", start.getX(),
              start.getY(), goal.getX(), goal.getY());
  return searchPath(start, goal);
}

nav_msgs::msg::Path HybridAStar::searchPath(Node3D& start, const Node3D& goal) {
  int idx_pred, idx_succ;
  int dir = constants::reverse ? 6 : 3;

  bool find_goal = false;
  int iterations = 0;
  std::priority_queue<std::shared_ptr<Node3D>,
                      std::vector<std::shared_ptr<Node3D>>, CompareNode3D>
      O;
  int width = map_->width();
  int height = map_->height();
  int depth = constants::headings;
  node3d_.clear();
  node3d_.resize(width * height * depth);

  updateH(start, goal);
  std::shared_ptr<Node3D> start_ptr = std::make_shared<Node3D>(start);

  start_ptr->open();
  O.push(start_ptr);

  idx_pred = start_ptr->setIdx(width, height);
  if (start_ptr->getT() < 0 || start_ptr->getT() >= 72) {
    RCLCPP_INFO(logger_, "start t: %f\n", start_ptr->getT());
  }
  node3d_[idx_pred] = start_ptr;
  std::shared_ptr<Node3D> node_pred, node_succ;

  while (!O.empty()) {
    node_pred = O.top();
    idx_pred = node_pred->setIdx(width, height);
    iterations++;
    O.pop();

    if (node_pred->getC() - epsilon > node3d_[idx_pred]->getC()) {
      continue;
    }

    node3d_[idx_pred]->close();

    if (*node_pred == goal || iterations > constants::iterations) {
      find_goal = *node_pred == goal;
      node_succ = node_pred;
      break;
    }
    if (constants::dubinsShot && node_pred->isInRange(goal) &&
        node_pred->getPrim() < 3) {
      node_succ = dubinsShot(node_pred, goal, configuration_space_);
      if (node_succ != nullptr) {
        find_goal = true;
        break;
      }
    }

    for (int i = 0; i < dir; ++i) {
      node_succ = node_pred->createSuccessor(i);
      idx_succ = node_succ->setIdx(width, height);
      if (node_succ->isOnGrid(width, height) &&
          configuration_space_.isTraversable(node_succ.get())) {
        node_succ->updateG();
        updateH(*node_succ, goal);
        double new_c = node_succ->getC();
        if (node3d_[idx_succ] == nullptr ||
            (!node3d_[idx_succ]->isOpen() && !node3d_[idx_succ]->isClosed())) {
          node_succ->open();
          node3d_[idx_succ] = node_succ;
          O.push(node_succ);
        } else if (node3d_[idx_succ]->isOpen() &&
                   new_c < node3d_[idx_succ]->getC()) {
          node_succ->open();
          node3d_[idx_succ] = node_succ;
        } else {
          continue;
        }
      }
    }
  }

  nav_msgs::msg::Path res_path;

  if (!find_goal) {
    std::cout << "[HybridAStar] No path found. iterations=" << iterations
              << ", O.empty=" << O.empty() << std::endl;
  }

  if (find_goal) {
    std::vector<std::shared_ptr<Node3D>> res_seq;
    int start_idx = start.setIdx(width, height);
    while (node_succ != nullptr) {
      res_seq.emplace_back(node_succ);
      if (node_succ->setIdx(width, height) == start_idx) {
        break;
      }
      node_succ = node_succ->getPred();
    }
    std::reverse(res_seq.begin(), res_seq.end());

    res_path.poses.reserve(res_seq.size());
    double wx, wy;
    for (std::shared_ptr<Node3D> node : res_seq) {
      map_->gridToWorld(node->getX(), node->getY(), wx, wy);
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = wx;
      p.pose.position.y = wy;
      p.pose.orientation.z = std::sin(node->getT() / 2.0);
      p.pose.orientation.w = std::cos(node->getT() / 2.0);
      res_path.poses.push_back(p);
    }
  }
  return res_path;
}

void HybridAStar::updateH(Node3D& start, const Node3D& goal) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;

  if (constants::dubins) {
    ompl::base::DubinsStateSpace dubinsPath(constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    dubinsPath.freeState(dbStart);
    dubinsPath.freeState(dbEnd);
  }

  if (constants::reverse && !constants::dubins) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    reedsSheppPath.freeState(rsStart);
    reedsSheppPath.freeState(rsEnd);
  }

  start.setH(std::max(reedsSheppCost, dubinsCost));
}

std::shared_ptr<Node3D> HybridAStar::dubinsShot(
    const std::shared_ptr<Node3D>& start, const Node3D& goal,
    common::CollisionDetection& configuration_space) {
  // start
  double q0[] = {start->getX(), start->getY(), start->getT()};
  // goal
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};
  // initialize the path
  DubinsPath path;
  // calculate the path
  int init_res = dubins_init(q0, q1, constants::r, &path);
  if (init_res != 0) {
    std::cout << "dubins_init failed: " << init_res << std::endl;
    return nullptr;
  }

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  std::vector<std::shared_ptr<Node3D>> dubinsNodes;
  dubinsNodes.resize((int)(length / constants::dubinsStepSize) + 1);

  // avoid duplicate waypoint
  x += constants::dubinsStepSize;
  while (x < length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    if (dubinsNodes[i] == nullptr) {
      dubinsNodes[i] = std::make_shared<Node3D>(
          q[0], q[1], normalizeHeadingRad(q[2]), 0, 0, nullptr);
    } else {
      dubinsNodes[i]->setX(q[0]);
      dubinsNodes[i]->setY(q[1]);
      dubinsNodes[i]->setT(normalizeHeadingRad(q[2]));
    }
    // collision check
    if (configuration_space.isTraversable(dubinsNodes[i].get())) {
      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i]->setPred(dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i]->setPred(start);
      }

      if (dubinsNodes[i] == dubinsNodes[i]->getPred()) {
        std::cout << "looping shot";
      }

      x += constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return dubinsNodes[i - 1];
}

}  // namespace global_planner