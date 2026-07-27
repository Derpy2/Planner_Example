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

  static bool point_cmp_x(const Node2D& a, const Node2D& b) {
    return a.x < b.x;
  }
  static bool point_cmp_y(const Node2D& a, const Node2D& b) {
    return a.y < b.y;
  }

  void build(std::vector<Node2D>& point_cloud) {
    if (root_ != nullptr) {
      deleteTreeNodes(&root_);
    }
    if (point_cloud.size() == 0) {
      return;
    }

    root_ = std::make_shared<KDTreeNode>();
    initTreeNode(root_);
    buildTree(&root_, 0, point_cloud.size() - 1, point_cloud);
    update(root_);
  }

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

  void runOperation(std::shared_ptr<KDTreeNode>& root,
                    const KDOperation& operation) {
    switch (operation.op) {
      case ADD_POINT:
        addByPoint(&root, operation.point, root->division_axis);
        break;
      case ADD_BOX:
        addByRange(&root, operation.rect);
        break;
      case DELETE_POINT:
        deleteByPoint(&root, operation.point);
        break;
      case DELETE_BOX:
        deleteByRange(&root, operation.rect);
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
            max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]),
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

  int deleteByRange(std::shared_ptr<KDTreeNode>* root, BoxPointType boxpoint) {
    if ((*root) == nullptr || (*root)->tree_deleted) {
      return 0;
    }

    pushDown(*root);

    int tmp_counter = 0;
    // 不在覆盖范围内
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] ||
        boxpoint.vertex_min[0] > (*root)->node_range_x[1]) {
      return 0;
    }
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] ||
        boxpoint.vertex_min[1] > (*root)->node_range_y[1]) {
      return 0;
    }
    // 覆盖整个node代表区域
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] &&
        boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] &&
        boxpoint.vertex_max[1] > (*root)->node_range_y[1]) {
      (*root)->tree_deleted = true;
      (*root)->point_deleted = true;
      (*root)->need_push_down_to_left = true;
      (*root)->need_push_down_to_right = true;
      tmp_counter = (*root)->tree_size - (*root)->invalid_point_num;
      (*root)->invalid_point_num = (*root)->tree_size;
      return tmp_counter;
    }
    // 判断当前node所对应的点
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x &&
        boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y &&
        boxpoint.vertex_max[1] > (*root)->point.y) {
      (*root)->point_deleted = true;
      tmp_counter += 1;
    }

    // 处理左右子节点
    tmp_counter += deleteByRange(&((*root)->left_child), boxpoint);
    tmp_counter += deleteByRange(&((*root)->right_child), boxpoint);
    update(*root);

    return tmp_counter;
  }
  void deleteByPoint(std::shared_ptr<KDTreeNode>* root, const Node2D& point) {
    if ((*root) == nullptr || (*root)->tree_deleted) {
      return;
    }

    pushDown(*root);

    if (samePoint((*root)->point, point) && !(*root)->point_deleted) {
      (*root)->point_deleted = true;
      (*root)->invalid_point_num += 1;
      if ((*root)->invalid_point_num == (*root)->tree_size) {
        (*root)->tree_deleted;
      }
      return;
    }

    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y)) {
      deleteByPoint(&(*root)->left_child, point);
    } else {
      deleteByPoint(&(*root)->right_child, point);
    }
    update(*root);
  }
  void addByPoint(std::shared_ptr<KDTreeNode>* root, const Node2D& point,
                  int father_axis) {
    if (*root == nullptr) {
      *root = std::make_shared<KDTreeNode>();
      initTreeNode(*root);
      (*root)->point = point;
      (*root)->division_axis = (father_axis + 1) % 2;
      update(*root);
      return;
    }

    pushDown(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y)) {
      addByPoint(&(*root)->left_child, point, (*root)->division_axis);
    } else {
      addByPoint(&(*root)->right_child, point, (*root)->division_axis);
    }

    update(*root);
  }
  void addByRange(std::shared_ptr<KDTreeNode>* root, BoxPointType boxpoint) {
    if ((*root) == nullptr) {
      return;
    }

    pushDown(*root);

    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] ||
        boxpoint.vertex_min[0] > (*root)->node_range_x[1]) {
      return;
    }
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] ||
        boxpoint.vertex_min[1] > (*root)->node_range_y[1]) {
      return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] &&
        boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] &&
        boxpoint.vertex_max[1] > (*root)->node_range_y[1]) {
      (*root)->tree_deleted = false;
      (*root)->point_deleted = false;
      (*root)->need_push_down_to_left = true;
      (*root)->need_push_down_to_right = true;
      (*root)->invalid_point_num = (*root)->down_del_num;
      return;
    }

    addByRange(&((*root)->left_child), boxpoint);
    addByRange(&((*root)->right_child), boxpoint);
    update(*root);
  }
  void search(const std::shared_ptr<KDTreeNode>& root, int k_nearest,
              const Node2D& point, ManualHeap& q, double max_dist) {
    if (root == nullptr || root->tree_deleted) {
      return;
    }

    double cur_dist = calcBoxDist(root, point);
    double max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr) {
      return;
    }

    if (root->need_push_down_to_left || root->need_push_down_to_right) {
      pushDown(root);
    }
    if (!root->point_deleted) {
      double dist = calcDist(point, root->point);
      if (dist <= max_dist_sqr &&
          (q.size() < k_nearest || dist < q.top().dist)) {
        if (q.size() >= k_nearest) q.pop();
        NodeCmp current_point{root->point, dist};
        q.push(current_point);
      }
    }

    double dist_left_node = calcBoxDist(root->left_child, point);
    double dist_right_node = calcBoxDist(root->right_child, point);

    if (q.size() < k_nearest ||
        (dist_left_node < q.top().dist && dist_right_node < q.top().dist)) {
      if (dist_left_node <= dist_right_node) {
        search(root->left_child, k_nearest, point, q, max_dist);
        if (q.size() < k_nearest || dist_right_node < q.top().dist) {
          search(root->right_child, k_nearest, point, q, max_dist);
        }
      } else {
        search(root->right_child, k_nearest, point, q, max_dist);
        if (q.size() < k_nearest || dist_left_node < q.top().dist) {
          search(root->left_child, k_nearest, point, q, max_dist);
        }
      }
    } else {
      if (dist_left_node < q.top().dist) {
        search(root->left_child, k_nearest, point, q, max_dist);
      }
      if (dist_right_node < q.top().dist) {
        search(root->right_child, k_nearest, point, q, max_dist);
      }
    }
  }
  void searchByRange(const std::shared_ptr<KDTreeNode>& root,
                     BoxPointType boxpoint, std::vector<Node2D>& result) {
    if (root == nullptr) {
      return;
    }

    pushDown(root);
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] ||
        boxpoint.vertex_min[0] > root->node_range_x[1]) {
      return;
    }
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] ||
        boxpoint.vertex_min[1] > root->node_range_y[1]) {
      return;
    }

    // 都在范围内时，直接跳过检查，加速搜索
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] &&
        boxpoint.vertex_max[0] > root->node_range_x[1] &&
        boxpoint.vertex_min[1] <= root->node_range_y[0] &&
        boxpoint.vertex_max[1] > root->node_range_y[1]) {
      flatten(root, result);
      return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x &&
        boxpoint.vertex_max[0] > root->point.x &&
        boxpoint.vertex_min[1] <= root->point.y &&
        boxpoint.vertex_max[1] > root->point.y) {
      if (!root->point_deleted) {
        result.push_back(root->point);
      }
    }

    searchByRange(root->left_child, boxpoint, result);
    searchByRange(root->right_child, boxpoint, result);
  }
  void searchByRadius(const std::shared_ptr<KDTreeNode>& root,
                      const Node2D& point, double radius,
                      std::vector<Node2D>& result) {
    if (root == nullptr) return;
    pushDown(root);

    Node2D range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;

    double dist = sqrt(calcDist(range_center, point));

    if (dist > radius + sqrt(root->radius_sq)) {
      return;
    }
    if (dist <= radius - sqrt(root->radius_sq)) {
      flatten(root, result);
      return;
    }

    if (!root->point_deleted &&
        calcDist(root->point, point) <= radius * radius) {
      result.push_back(root->point);
    }
    searchByRadius(root->left_child, point, radius, result);
    searchByRadius(root->right_child, point, radius, result);
  }

  void deleteTreeNodes(std::shared_ptr<KDTreeNode>* root) {
    if (*root == nullptr) return;

    pushDown(*root);
    if ((*root)->left_child != nullptr) {
      deleteTreeNodes(&(*root)->left_child);
    }
    if ((*root)->right_child != nullptr) {
      deleteTreeNodes(&(*root)->right_child);
    }
    *root = nullptr;
  }
  bool samePoint(const Node2D& a, const Node2D& b) {
    return (fabs(a.x - b.x) < epsilon && fabs(a.y - b.y) < epsilon);
  }
  double calcDist(const Node2D& a, const Node2D& b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
  }
  double calcBoxDist(const std::shared_ptr<KDTreeNode>& node,
                     const Node2D& point) {
    if (node == nullptr) return INFINITY;
    double min_dist = 0.0;
    if (point.x < node->node_range_x[0])
      min_dist +=
          (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
      min_dist +=
          (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0])
      min_dist +=
          (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
      min_dist +=
          (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    return min_dist;
  }

  void nearestSearch(const Node2D& point, int k_nearest,
                     std::vector<Node2D>& nearest_points,
                     std::vector<double>& point_distance,
                     double max_dist = INFINITY) {
    ManualHeap q(2 * k_nearest);
    q.clear();
    vector<double>().swap(point_distance);
    search(root_, k_nearest, point, q, max_dist);

    int k_found = min(k_nearest, static_cast<int>(q.size()));
    vector<Node2D>().swap(nearest_points);
    vector<double>().swap(point_distance);
    for (int i = 0; i < k_found; ++i) {
      nearest_points.insert(nearest_points.begin(), q.top().point);
      point_distance.insert(point_distance.begin(), q.top().dist);
    }
  }
  void boxSearch(const BoxPointType& box_of_point,
                 std::vector<Node2D>& result) {
    result.clear();
    searchByRange(root_, box_of_point, result);
    return;
  }
  void radiusSearch(const Node2D& point, const double radius,
                    std::vector<Node2D>& result) {
    result.clear();
    searchByRadius(root_, point, radius, result);
    return;
  }
  void addPoints(const std::vector<Node2D>& point_to_add) {
    for (size_t i = 0; i < point_to_add.size(); i++) {
      addByPoint(&root_, point_to_add[i], root_->division_axis);
    }
    return;
  }
  void addPointBoxes(std::vector<BoxPointType>& box_points) {
    for (size_t i = 0; i < box_points.size(); i++) {
      addByRange(&root_, box_points[i]);
    }
    return;
  }
  void deletePoints(const std::vector<Node2D>& point_to_del) {
    for (size_t i = 0; i < point_to_del.size(); i++) {
      deleteByPoint(&root_, point_to_del[i]);
    }
    return;
  }
  int deletePointBoxes(std::vector<BoxPointType>& box_points) {
    int tmp_counter = 0;
    for (size_t i = 0; i < box_points.size(); i++) {
      tmp_counter += deleteByRange(&root_, box_points[i]);
    }
    return tmp_counter;
  }

  void pushDown(const std::shared_ptr<KDTreeNode>& root) {
    if (root == nullptr) {
      return;
    }

    if (root->need_push_down_to_left && root->left_child != nullptr) {
      root->left_child->tree_deleted = root->tree_deleted;
      root->left_child->point_deleted = root->left_child->tree_deleted;
      if (root->tree_deleted) {
        root->left_child->invalid_point_num = root->left_child->tree_size;
      } else {
        root->left_child->invalid_point_num = root->left_child->down_del_num;
      }
      root->left_child->need_push_down_to_left = true;
      root->left_child->need_push_down_to_right = true;
      root->need_push_down_to_left = false;
    }

    if (root->need_push_down_to_right && root->right_child != nullptr) {
      root->right_child->tree_deleted = root->tree_deleted;
      root->right_child->point_deleted = root->right_child->tree_deleted;
      if (root->tree_deleted) {
        root->right_child->invalid_point_num = root->right_child->tree_size;
      } else {
        root->right_child->invalid_point_num = root->right_child->down_del_num;
      }
      root->right_child->need_push_down_to_left = true;
      root->right_child->need_push_down_to_right = true;
      root->need_push_down_to_right = false;
    }
  }

  void flatten(const std::shared_ptr<KDTreeNode>& root,
               std::vector<Node2D>& result) {
    if (root == nullptr) return;
    pushDown(root);
    if (!root->point_deleted) {
      result.push_back(root->point);
    }

    flatten(root->left_child, result);
    flatten(root->right_child, result);
  }

 private:
  std::shared_ptr<KDTreeNode> root_ = nullptr;
};

}  // namespace map