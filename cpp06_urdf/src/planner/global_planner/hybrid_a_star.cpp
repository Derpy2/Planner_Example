#include "planner/global_planner/hybrid_a_star.h"

#include <cmath>
#include <queue>
#include <vector>

#include "common/config.h"
#include "common/util.h"

namespace global_planner {
using namespace common;

struct CompareNode3D {
  bool operator()(const std::shared_ptr<Node3D> lhs,
                  const std::shared_ptr<Node3D> rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

nav_msgs::msg::Path HybridAStar::searchPath(Node3D& start, const Node3D& goal) {
  int idx_pred, idx_succ;
  float new_g;
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

  start.open();
  O.push(std::make_shared<Node3D>(start));

  idx_pred = start.setIdx(width, height);
  node3d_[idx_pred] = std::make_shared<Node3D>(start);

  std::shared_ptr<Node3D> node_pred, node_succ;

  while (!O.empty()) {
    node_pred = std::make_shared<Node3D>(O.top());
    idx_pred = node_pred->setIdx(width, height);
    iterations++;

    if (node3d_[idx_pred]->isClosed()) {
      O.pop();
      continue;
    } else if (node3d_[idx_pred]->isOpen()) {
      node3d_[idx_pred]->close();

      O.pop();

      if (*node_pred == goal || iterations > constants::iterations) {
        find_goal = *node_pred == goal;
        break;
        // return node_pred;
      }

      if (constants::dubinsShot && node_pred->isInRange(goal) &&
          node_pred->getPrim() < 3) {
        node_succ = dubinsShot(*node_pred, goal, configuration_space_);

        if (node_succ != nullptr && *node_succ == goal) {
          find_goal = *node_pred == goal;
          break;
        }
      }

      for (int i = 0; i < dir; ++i) {
        node_succ = std::make_shared<Node3D>(node_pred->createSuccessor(i));
        idx_succ = node_succ->setIdx(width, height);

        if (node_succ->isOnGrid(width, height) &&
            configuration_space_.isTraversable(node_succ.get())) {
          node_succ->updateG();
          new_g = node_succ->getG();
          if (!node3d_[idx_succ]->isOpen() && !node3d_[idx_succ]->isClosed()) {
            updateH(*node_succ, goal);
            node_succ->open();
            node3d_[idx_succ] = node_succ;
            O.push(node_succ);
          } else if (node3d_[idx_succ]->isClosed() &&
                     new_g < node3d_[idx_succ]->getG()) {
            updateH(*node_succ, goal);
            node_succ->open();
            node3d_[idx_succ] = node_succ;
            O.push(node_succ);
          } else if (node3d_[idx_succ]->isOpen() &&
                     new_g < node3d_[idx_succ]->getG()) {
            updateH(*node_succ, goal);
            node_succ->open();
            node3d_[idx_succ] = node_succ;
          } else {
            continue;
          }
        }
      }
    }
  }

  nav_msgs::msg::Path res_path;

  if (find_goal) {
    std::vector<std::shared_ptr<Node3D>> res_seq;
    int start_idx = start.setIdx(width, height);
    while (node_succ != nullptr) {
      res_seq.emplace_back(node_succ);
      if (node_succ->setIdx(width, height) == start_idx) {
        break;
      }
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
  }

  start.setH(std::max(reedsSheppCost, dubinsCost));
}

std::shared_ptr<Node3D> dubinsShot(
    Node3D& start, const Node3D& goal,
    common::CollisionDetection& configuration_space) {
  // start
  double q0[] = {start.getX(), start.getY(), start.getT()};
  // goal
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};
  // initialize the path
  DubinsPath path;
  // calculate the path
  int init_res = 0;
  if (!(init_res = dubins_init(q0, q1, constants::r, &path))) {
    std::cout << "dubins_init failed: " << init_res << std::endl;
    return nullptr;
  }

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  std::vector<Node3D> dubinsNodes;
  dubinsNodes.resize((int)(length / constants::dubinsStepSize) + 1);
  // Node3D* dubinsNodes =
  //     new Node3D[(int)(length / constants::dubinsStepSize) + 1];

  // avoid duplicate waypoint
  x += constants::dubinsStepSize;
  while (x < length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(normalizeHeadingRad(q[2]));

    // collision check
    if (configuration_space.isTraversable(&dubinsNodes[i])) {
      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
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
  return std::make_shared<Node3D>(dubinsNodes[i - 1]);
}

}  // namespace global_planner