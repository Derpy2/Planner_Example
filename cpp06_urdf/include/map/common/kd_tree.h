#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>

namespace map {

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
  Vector2d point;
  BoxPointType rect;
  bool tree_deleted;
};

struct KDTreeNode {
  int val = 0;
  int node_range_x[2], node_range_y[2];
  int division_axis;
  int tree_size;
  bool need_push_down_to_left;
  bool need_push_down_to_right;
  bool tree_deleted, point_deleted;
  int invalid_point_num;
  int down_del_num;
  std::shared_ptr<KDTreeNode> father;
  std::shared_ptr<KDTreeNode> left_child;
  std::shared_ptr<KDTreeNode> right_child;
};

class KDTree {
 public:
  KDTree() {}

  void initTreeNode(const std::shared_ptr<KDTreeNode>& root) {
    root->val = 0;
    root->node_range_x[0] = 0.0;
    root->node_range_x[1] = 0.0;
    root->node_range_y[0] = 0.0;
    root->node_range_y[1] = 0.0;

    root->division_axis = 0;
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

  int deleteByRange(const std::shared_ptr<KDTreeNode>& root,
                    BoxPointType boxpoint);
  void deleteByPoint(const std::shared_ptr<KDTreeNode>& root,
                     const Vector2d& point);
  void addByPoint(const std::shared_ptr<KDTreeNode>& root,
                  const Vector2d& point, bool allow_rebuild, int father_axis);
  void addByRange(const std::shared_ptr<KDTreeNode>& root,
                  BoxPointType boxpoint, bool allow_rebuild);
  void search(const std::shared_ptr<KDTreeNode>& root, int k_nearest,
              const Vector2d& point, double max_dist);
  void searchByRange(const std::shared_ptr<KDTreeNode>& root,
                     BoxPointType boxpoint, std::vector<Vector2d>& Storage);
  void searchByRadius(const std::shared_ptr<KDTreeNode>& root,
                      const Vector2d& point, float radius,
                      std::vector<Vector2d>& Storage);
  void update(const std::shared_ptr<KDTreeNode>& root);
  void deleteTreeNodes(const std::shared_ptr<KDTreeNode>& root);
  bool samePoint(const Vector2d& a, const Vector2d& b);
  float calcDist(const Vector2d& a, const Vector2d& b);
  float calcBoxDist(const std::shared_ptr<KDTreeNode>& node,
                    const Vector2d& point);

  void nearestSearch(const Vector2d& point, int k_nearest,
                     std::vector<Vector2d>& Nearest_Points,
                     std::vector<double>& Point_Distance,
                     double max_dist = INFINITY);
  void boxSearch(const BoxPointType& Box_of_Point,
                 std::vector<Vector2d>& Storage);
  void radiusSearch(const Vector2d& point, const float radius,
                    std::vector<Vector2d>& Storage);
  int addPoints(const std::vector<Vector2d>& PointToAdd);
  void addPointBoxes(std::vector<BoxPointType>& BoxPoints);
  void deletePoints(const std::vector<Vector2d>& PointToDel);
  int deletePointBoxes(std::vector<BoxPointType>& BoxPoints);

 private:
  std::shared_ptr<KDTreeNode> root = nullptr;
};

}  // namespace map