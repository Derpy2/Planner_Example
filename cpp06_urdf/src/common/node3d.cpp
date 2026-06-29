#include "common/node3d.h"

#include <random>

#include "common/util.h"

namespace common {
const int Node3D::dir = 3;
// R = 6, 6.75 DEG
const float Node3D::dy[] = {0, -0.0415893, 0.0415893};
const float Node3D::dx[] = {0.7068582, 0.705224, 0.705224};
const float Node3D::dt[] = {0, 0.1178097, -0.1178097};

bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height &&
         (int)(t / constants::deltaHeadingRad) >= 0 &&
         (int)(t / constants::deltaHeadingRad) < constants::headings;
}

bool Node3D::isInRange(const Node3D& goal) const {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distribution(1, 10);
  int random = distribution(gen);
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < constants::dubinsShotDistance;
}

Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc;

  // calculate successor positions forward
  if (i < 3) {
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    tSucc = normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
    ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
    tSucc = normalizeHeadingRad(t - dt[i - 3]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
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