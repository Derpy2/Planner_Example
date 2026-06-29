#include "common/collision_detection.h"

namespace common {

CollisionDetection::CollisionDetection(shared_ptr<map::StaticMap> map)
    : map_(map) {}

std::vector<Node3D> CollisionDetection::GetVehiclePolygon(float x, float y,
                                                          float t, float width,
                                                          float length) {
  double wx, wy;
  map_->gridToWorld(x, y, wx, wy);

  // world cord xy
  std::vector<Node3D> polygon;
  float half_l = length / 2.0f;
  float half_w = width / 2.0f;

  // local corners: front-left, front-right, rear-right, rear-left
  float local_x[4] = {half_l, half_l, -half_l, -half_l};
  float local_y[4] = {half_w, -half_w, -half_w, half_w};

  float cos_t = std::cos(t);
  float sin_t = std::sin(t);
  int gx, gy;
  for (int i = 0; i < 4; ++i) {
    float rx = local_x[i] * cos_t - local_y[i] * sin_t + wx;
    float ry = local_x[i] * sin_t + local_y[i] * cos_t + wy;
    map_->worldToGrid(rx, ry, gx, gy);
    polygon.emplace_back(gx, gy, t, 0, 0, nullptr);
  }
  // grid polygon
  return polygon;
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
  std::vector<Node3D> vehicle_polygon = GetVehiclePolygon(
      x, y, t, constants::vehicle_width, constants::vehicle_length);
  int max_x = vehicle_polygon[0].x(), min_x = vehicle_polygon[0].x();
  int max_y = vehicle_polygon[0].y(), min_y = vehicle_polygon[0].y();
  for (const auto& node : vehicle_polygon) {
    // inBound
    if (!map_->inBounds(node.x(), node.y())) {
      return false;
    }
    max_x = std::max(max_x, node.x());
    min_x = std::min(min_x, node.x());
    max_y = std::max(max_y, node.y());
    min_y = std::min(min_y, node.y());
  }
  const auto& map_msg = map_->getMapMsg();
  for (int i = min_x; i <= max_x; ++i) {
    for (int j = min_y; j <= max_y; ++j) {
      if (map_msg.data[map_->idx(i, j)] >= 50) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace common