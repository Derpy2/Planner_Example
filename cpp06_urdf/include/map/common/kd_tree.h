#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>

#include "common/node2d.h"

namespace map {

using namespace std;
using namespace common;
using namespace Eigen;
// 为二维栅格地图准备的二维KDTree, 支持

struct LazyInfo {};

enum KDOperationType {
  ADD_POINT = 0,
  ADD_BOX = 1,
  DELETE_POINT = 2,
  DELETE_BOX = 3,
  PUSH_DOWN = 4
};

struct BoxPointType {
  int vertex_min[2];
  int vertex_max[2];
};

struct KDOperation {
  KDOperationType op;
  Node2D point;
  BoxPointType rect;
  bool tree_deleted;
};

struct KDTreeNode {
  Node2D point = 0;
  double node_range_x[2], node_range_y[2];
  int division_axis;
  int tree_size;
  bool need_push_down_to_left;
  bool need_push_down_to_right;
  bool tree_deleted, point_deleted;
  int invalid_point_num;
  int down_del_num;
  double radius_sq;
  std::shared_ptr<KDTreeNode> father;
  std::shared_ptr<KDTreeNode> left_child;
  std::shared_ptr<KDTreeNode> right_child;
};

class KDTree {
 public:
  KDTree() {}

  void initTreeNode(const std::shared_ptr<KDTreeNode>& root) {
    root->point = Node2D(0.0, 0.0);
    root->node_range_x[0] = 0.0;
    root->node_range_x[1] = 0.0;
    root->node_range_y[0] = 0.0;
    root->node_range_y[1] = 0.0;

    root->division_axis = 0;
    root->radius_sq = 0;
    root->father = nullptr;
    root->left_child = nullptr;
    root->right_child = nullptr;
    root->tree_size = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
  }

  bool point_cmp_x(Node2D& a, Node2D& b) { return a.x < b.x; }
  bool point_cmp_y(Node2D& a, Node2D& b) { return a.y < b.y; }

  void buildTree(std::shared_ptr<KDTreeNode>* root, int l, int r,
                 std::vector<common::Node2D>& point_cloud) {
    if (l > r) return;
    *root = std::make_shared<KDTreeNode>();
    initTreeNode(*root);
    int mid = (l + r) >> 1;
    int div_axis = 0;

    // 寻找差值最大的轴
    double min_value[2] = {INFINITY, INFINITY};
    double max_value[2] = {-INFINITY, -INFINITY};
    double dim_range[2] = {0, 0};
    for (int i = l; i <= r; ++i) {
      min_value[0] = min(min_value[0], point_cloud[i].x);
      min_value[1] = min(min_value[1], point_cloud[i].y);
      max_value[0] = max(max_value[0], point_cloud[i].x);
      max_value[1] = max(max_value[1], point_cloud[i].y);
    }

    for (int i = 0; i < 2; ++i) {
      dim_range[i] = max_value[i] - min_value[i];
    }
    if (dim_range[1] > dim_range[0]) {
      div_axis = 1;
    } else {
      div_axis = 0;
    }

    // 按照选择的轴重新排列障碍物点云
    (*root)->division_axis = div_axis;
    switch (div_axis) {
      case 0: {
        nth_element(begin(point_cloud) + l, begin(point_cloud) + mid,
                    begin(point_cloud) + r + 1, point_cmp_x);
        break;
      }
      case 1: {
        nth_element(begin(point_cloud) + l, begin(point_cloud) + mid,
                    begin(point_cloud) + r + 1, point_cmp_y);
        break;
      }
      default: {
        nth_element(begin(point_cloud) + l, begin(point_cloud) + mid,
                    begin(point_cloud) + r + 1, point_cmp_x);
        break;
      }
    }

    // 继续建子树
    (*root)->point = point_cloud[mid];
    std::shared_ptr<KDTreeNode> left_son = nullptr, right_son = nullptr;
    buildTree(&left_son, l, mid - 1, point_cloud);
    buildTree(&right_son, mid + 1, r, point_cloud);
    (*root)->left_child = left_son;
    (*root)->right_child = right_son;
    update((*root));
  }

  void runOperation(const std::shared_ptr<KDTreeNode>& root,
                    const KDOperation& operation) {
    switch (operation.op) {
      case ADD_POINT:
        addByPoint(root, operation.point, root->division_axis);
        break;
      case ADD_BOX:
        addByRange(root, operation.rect);
        break;
      case DELETE_POINT:
        deleteByPoint(root, operation.point);
        break;
      case DELETE_BOX:
        deleteByRange(root, operation.rect);
        break;
      case PUSH_DOWN:
        root->tree_deleted = operation.tree_deleted;
        root->point_deleted = root->tree_deleted;
        if (operation.tree_deleted) {
          root->invalid_point_num = root->tree_size;
        } else {
          root->invalid_point_num = root->down_del_num;
        }
        root->need_push_down_to_left = true;
        root->need_push_down_to_right = true;
        break;
      default:
        break;
    }
  }

  void update(const std::shared_ptr<KDTreeNode>& root) {
    std::shared_ptr<KDTreeNode> left_son_ptr = root->left_child;
    std::shared_ptr<KDTreeNode> right_son_ptr = root->right_child;

    double tmp_range_x[2] = {INFINITY, INFINITY};
    double tmp_range_y[2] = {INFINITY, INFINITY};

    if (left_son_ptr != nullptr && right_son_ptr != nullptr) {
      root->tree_size = left_son_ptr->tree_size + right_son_ptr->tree_size + 1;
      root->invalid_point_num = left_son_ptr->invalid_point_num +
                                right_son_ptr->invalid_point_num +
                                (root->point_deleted ? 1 : 0);
      root->down_del_num =
          left_son_ptr->down_del_num + right_son_ptr->down_del_num;
      root->tree_deleted = left_son_ptr->tree_deleted &&
                           right_son_ptr->tree_deleted && root->point_deleted;
      if (root->tree_deleted ||
          (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted &&
           !root->point_deleted)) {
        tmp_range_x[0] = min(
            min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]),
            root->point.x);
        tmp_range_x[1] = max(
            max(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]),
            root->point.x);
        tmp_range_y[0] = min(
            min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]),
            root->point.y);
        tmp_range_y[1] = max(
            max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]),
            root->point.y);
      } else {
        if (!left_son_ptr->tree_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
          tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
          tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
          tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
        }
        if (!right_son_ptr->tree_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
          tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
          tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
          tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
        }

        if (!root->point_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
          tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
          tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
          tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
        }
      }
    } else if (left_son_ptr != nullptr) {
      root->tree_size = left_son_ptr->tree_size + 1;
      root->invalid_point_num =
          left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
      root->down_del_num = left_son_ptr->down_del_num;
      root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
      if (root->tree_deleted ||
          (!left_son_ptr->tree_deleted && !root->point_deleted)) {
        tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
        tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
        tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
        tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
      } else {
        if (!left_son_ptr->tree_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
          tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
          tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
          tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
        }
        if (!root->point_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
          tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
          tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
          tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
        }
      }
    } else if (right_son_ptr != nullptr) {
      root->tree_size = right_son_ptr->tree_size + 1;
      root->invalid_point_num =
          right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
      root->down_del_num = right_son_ptr->down_del_num;

      root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
      if (root->tree_deleted ||
          (!right_son_ptr->tree_deleted && !root->point_deleted)) {
        tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
        tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
        tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
        tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
      } else {
        if (!right_son_ptr->tree_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
          tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
          tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
          tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
        }
        if (!root->point_deleted) {
          tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
          tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
          tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
          tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
        }
      }

    } else {
      root->tree_size = 1;
      root->invalid_point_num = (root->point_deleted ? 1 : 0);
      root->tree_deleted = root->point_deleted;
      tmp_range_x[0] = root->point.x;
      tmp_range_x[1] = root->point.x;
      tmp_range_y[0] = root->point.y;
      tmp_range_y[1] = root->point.y;
    }

    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    double x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    double y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;

    root->radius_sq = x_L * x_L + y_L * y_L;
    if (left_son_ptr != nullptr) left_son_ptr->father = root;
    if (right_son_ptr != nullptr) right_son_ptr->father = root;
    return;
  }

  int deleteByRange(const std::shared_ptr<KDTreeNode>& root,
                    BoxPointType boxpoint);
  void deleteByPoint(const std::shared_ptr<KDTreeNode>& root,
                     const Node2D& point);
  void addByPoint(const std::shared_ptr<KDTreeNode>& root, const Node2D& point,
                  int father_axis);
  void addByRange(const std::shared_ptr<KDTreeNode>& root,
                  BoxPointType boxpoint);
  void search(const std::shared_ptr<KDTreeNode>& root, int k_nearest,
              const Node2D& point, double max_dist);
  void searchByRange(const std::shared_ptr<KDTreeNode>& root,
                     BoxPointType boxpoint, std::vector<Node2D>& Storage);
  void searchByRadius(const std::shared_ptr<KDTreeNode>& root,
                      const Node2D& point, float radius,
                      std::vector<Node2D>& Storage);

  void deleteTreeNodes(const std::shared_ptr<KDTreeNode>& root);
  bool samePoint(const Node2D& a, const Node2D& b);
  float calcDist(const Node2D& a, const Node2D& b);
  float calcBoxDist(const std::shared_ptr<KDTreeNode>& node,
                    const Node2D& point);

  void nearestSearch(const Node2D& point, int k_nearest,
                     std::vector<Node2D>& Nearest_Points,
                     std::vector<double>& Point_Distance,
                     double max_dist = INFINITY);
  void boxSearch(const BoxPointType& Box_of_Point,
                 std::vector<Node2D>& Storage);
  void radiusSearch(const Node2D& point, const float radius,
                    std::vector<Node2D>& Storage);
  int addPoints(const std::vector<Node2D>& PointToAdd);
  void addPointBoxes(std::vector<BoxPointType>& BoxPoints);
  void deletePoints(const std::vector<Node2D>& PointToDel);
  int deletePointBoxes(std::vector<BoxPointType>& BoxPoints);

 private:
  std::shared_ptr<KDTreeNode> root = nullptr;
};

}  // namespace map