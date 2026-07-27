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

enum KDOperationType {
  ADD_POINT = 0,
  ADD_BOX = 1,
  DELETE_POINT = 2,
  DELETE_BOX = 3,
  PUSH_DOWN = 4
};

struct BoxPointType {
  double vertex_min[2];
  double vertex_max[2];
};

struct KDOperation {
  KDOperationType op;
  Node2D point;
  BoxPointType rect;
  bool tree_deleted;
};

struct NodeCmp {
  Node2D point;
  double dist = 0.0;
  NodeCmp(Node2D p = Node2D(), double d = INFINITY) {
    point = p;
    dist = d;
  };
  bool operator<(const NodeCmp& other) const {
    if (fabs(dist - other.dist) < epsilon) {
      return point.x < other.point.x;
    } else {
      return dist < other.dist;
    }
  }
};

class ManualHeap {
 public:
  ManualHeap(int max_capacity = 100) {
    cap = max_capacity;
    heap = new NodeCmp[max_capacity];
    heap_size = 0;
  }

  ~ManualHeap() { delete[] heap; }

  void pop() {
    if (heap_size == 0) {
      return;
    }
    heap[0] = heap[heap_size - 1];
    heap_size--;
    moveDown(0);
  }

  NodeCmp top() { return heap[0]; }

  void push(NodeCmp point) {
    if (heap_size >= cap) {
      return;
    }

    heap[heap_size] = point;
    floatUp(heap_size);
    heap_size++;
    return;
  }

  int size() { return heap_size; }

  void clear() { heap_size = 0; }

 private:
  int heap_size = 0;
  int cap = 0;
  NodeCmp* heap;
  void moveDown(int heap_idx) {
    int l = heap_idx * 2 + 1;
    NodeCmp tmp = heap[heap_idx];
    while (l < heap_size) {
      if (l + 1 < heap_size && heap[l] < heap[l + 1]) {
        l++;
      }
      if (tmp < heap[l]) {
        heap[heap_idx] = heap[l];
        heap_idx = l;
        l = heap_idx * 2 + 1;
      } else {
        break;
      }
    }
    heap[heap_idx] = tmp;
    return;
  }

  void floatUp(int heap_idx) {
    int ancestor = (heap_idx - 1) / 2;
    NodeCmp tmp = heap[heap_idx];
    while (heap_idx > 0) {
      if (heap[ancestor] < tmp) {
        heap[heap_idx] = heap[ancestor];
        heap_idx = ancestor;
        ancestor = (heap_idx - 1) / 2;
      } else {
        break;
      }
    }
    heap[heap_idx] = tmp;
    return;
  }
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

  void initTreeNode(const std::shared_ptr<KDTreeNode>& root);

  static bool point_cmp_x(const Node2D& a, const Node2D& b) {
    return a.x < b.x;
  }
  static bool point_cmp_y(const Node2D& a, const Node2D& b) {
    return a.y < b.y;
  }

  void build(std::vector<Node2D>& point_cloud);
  void buildTree(std::shared_ptr<KDTreeNode>* root, int l, int r,
                 std::vector<common::Node2D>& point_cloud);
  void runOperation(std::shared_ptr<KDTreeNode>& root,
                    const KDOperation& operation);
  void update(const std::shared_ptr<KDTreeNode>& root);
  int deleteByRange(std::shared_ptr<KDTreeNode>* root, BoxPointType boxpoint);
  void deleteByPoint(std::shared_ptr<KDTreeNode>* root, const Node2D& point);
  void addByPoint(std::shared_ptr<KDTreeNode>* root, const Node2D& point,
                  int father_axis);
  void addByRange(std::shared_ptr<KDTreeNode>* root, BoxPointType boxpoint);
  void search(const std::shared_ptr<KDTreeNode>& root, int k_nearest,
              const Node2D& point, ManualHeap& q, double max_dist);
  void searchByRange(const std::shared_ptr<KDTreeNode>& root,
                     BoxPointType boxpoint, std::vector<Node2D>& result);
  void searchByRadius(const std::shared_ptr<KDTreeNode>& root,
                      const Node2D& point, double radius,
                      std::vector<Node2D>& result);
  void deleteTreeNodes(std::shared_ptr<KDTreeNode>* root);
  bool samePoint(const Node2D& a, const Node2D& b);
  double calcDist(const Node2D& a, const Node2D& b);
  double calcBoxDist(const std::shared_ptr<KDTreeNode>& node,
                     const Node2D& point);
  void nearestSearch(const Node2D& point, int k_nearest,
                     std::vector<Node2D>& nearest_points,
                     std::vector<double>& point_distance,
                     double max_dist = INFINITY);
  void boxSearch(const BoxPointType& box_of_point, std::vector<Node2D>& result);
  void radiusSearch(const Node2D& point, const double radius,
                    std::vector<Node2D>& result);
  void addPoints(const std::vector<Node2D>& point_to_add);
  void addPointBoxes(std::vector<BoxPointType>& box_points);
  void deletePoints(const std::vector<Node2D>& point_to_del);
  int deletePointBoxes(std::vector<BoxPointType>& box_points);
  void pushDown(const std::shared_ptr<KDTreeNode>& root);
  void flatten(const std::shared_ptr<KDTreeNode>& root,
               std::vector<Node2D>& result);

 private:
  std::shared_ptr<KDTreeNode> root_ = nullptr;
};

}  // namespace map