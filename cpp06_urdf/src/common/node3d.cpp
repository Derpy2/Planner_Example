#include "common/node3d.h"

#include <random>

#include "common/util.h"

namespace common {
const int Node3D::dir = 3;
// rad = 5 DEG
const float Node3D::dx[] = {0.087266463, 0.087155743, 0.087155743};
const float Node3D::dy[] = {0, 0.003805302, -0.003805302};
const float Node3D::dt[] = {0, 0.087266463, -0.087266463};
//   L            R             S
// x 0.087155743  0.087155743   0.087266463
// y 0.003805302  -0.003805302  0
// t 0.087266463  -0.087266463  0
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height &&
         (int)(t / constants::deltaHeadingRad) >= 0 &&
         (int)(t / constants::deltaHeadingRad) < constants::headings;
}

bool Node3D::isInRange(const Node3D& goal) const {
  float dx = std::abs(x - goal.x);
  float dy = std::abs(y - goal.y);
  return (dx * dx) + (dy * dy) < constants::dubinsShotDistance;
}

std::shared_ptr<Node3D> Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;
  float R = 12.0;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + R * (dx[i] * cos(t) - dy[i] * sin(t));
    ySucc = y + R * (dx[i] * sin(t) + dy[i] * cos(t));
    tSucc = normalizeHeadingRad(t + dt[i]);
  } else {
    // backwards
    xSucc = x - R * (dx[i - 3] * cos(t) + dy[i - 3] * sin(t));
    ySucc = y - R * (dx[i - 3] * sin(t) - dy[i - 3] * cos(t));
    tSucc = normalizeHeadingRad(t - dt[i - 3]);
  }
  std::shared_ptr<Node3D> node = std::make_shared<Node3D>(
      xSucc, ySucc, tSucc, g, 0, shared_from_this(), i);
  return node;
}

void Node3D::updateG() {
  // forward driving
  if (prim < 3) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > 2) {
        g += dx[0] * constants::penaltyTurning * constants::penaltyCOD;
      } else {
        g += dx[0] * constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < 3) {
        g += dx[0] * constants::penaltyTurning * constants::penaltyReversing *
             constants::penaltyCOD;
      } else {
        g += dx[0] * constants::penaltyTurning * constants::penaltyReversing;
      }
    } else {
      g += dx[0] * constants::penaltyReversing;
    }
  }
}

bool Node3D::operator==(const Node3D& rhs) const {
  return (int)x == (int)rhs.x && (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= constants::deltaHeadingNegRad);
}

}  // namespace common