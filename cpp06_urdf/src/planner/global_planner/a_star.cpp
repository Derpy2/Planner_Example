#include "planner/global_planner/a_star.h"

#include <limits>

namespace global_planner {

nav_msgs::msg::Path AStar::searchPath(const double sx, const double sy,
                                      const double gx, const double gy) {
  nav_msgs::msg::Path path;
  int sgx, sgy, ggx, ggy;
  map_->worldToGrid(sx, sy, sgx, sgy);
  map_->worldToGrid(gx, gy, ggx, ggy);
  const auto& map_msg = map_->getMapMsg();
  if (!map_->inBounds(sgx, sgy) || !map_->inBounds(ggx, ggy)) {
    RCLCPP_WARN(logger_, "Start or goal out of map bounds.");
    return path;
  }

  if (map_msg.data[map_->idx(sgx, sgy)] >= 50 ||
      map_msg.data[map_->idx(ggx, ggy)] >= 50) {
    RCLCPP_WARN(logger_, "Start or goal lies on an obstacle.");
    return path;
  }

  const int N = (int)map_->width() * (int)map_->height();
  std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(N, -1);
  std::vector<bool> closed(N, false);
  auto h = [&](int x, int y) {
    double dx = x - ggx, dy = y - ggy;
    return std::hypot(dx, dy);
  };
  std::priority_queue<AStarNode, std::vector<AStarNode>,
                      std::greater<AStarNode>>
      open;
  gscore[map_->idx(sgx, sgy)] = 0.0;
  open.push({sgx, sgy, h(sgx, sgy)});
  bool found = false;
  while (!open.empty()) {
    AStarNode cur = open.top();
    open.pop();
    int ci = map_->idx(cur.x, cur.y);
    if (closed[ci]) continue;
    closed[ci] = true;
    if (cur.x == ggx && cur.y == ggy) {
      found = true;
      break;
    }
    for (int k = 0; k < 8; ++k) {
      int nx = cur.x + dxs[k];
      int ny = cur.y + dys[k];
      if (!map_->inBounds(nx, ny)) continue;
      int ni = map_->idx(nx, ny);
      if (map_msg.data[ni] >= 50) continue;
      double tentative = gscore[ci] + costs[k] * map_->resolution();
      if (tentative < gscore[ni]) {
        gscore[ni] = tentative;
        came_from[ni] = ci;
        open.push({nx, ny, tentative + h(nx, ny) * map_->resolution()});
      }
    }
  }
  if (!found) {
    RCLCPP_WARN(logger_, "A* failed to find a path.");
    return path;
  }
  std::vector<int> rev;
  for (int cur = map_->idx(ggx, ggy); cur != -1; cur = came_from[cur]) {
    rev.push_back(cur);
  }
  std::reverse(rev.begin(), rev.end());

  path.poses.reserve(rev.size());
  for (int i : rev) {
    int x = i % (int)map_->width();
    int y = i / (int)map_->width();
    double wx, wy;
    map_->gridToWorld(x, y, wx, wy);
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace global_planner