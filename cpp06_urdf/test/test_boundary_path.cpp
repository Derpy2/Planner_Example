#include <gtest/gtest.h>

#include <iostream>

#include "common/node2d.h"
#include "planner/global_planner/complete_cover_path/boundary_path.h"

using namespace common;
using namespace global_planner::complete_cover_path;

TEST(BoundaryPathTest, SingleSquareObstacleZeroOffset) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(10, 0), Node2D(10, 10),
                        Node2D(0, 10)};
  BoundaryPathOptions opts{0.0, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GE(path.size(), 4u);
  for (const auto& p : path) {
    EXPECT_GE(p.x, -1e-6);
    EXPECT_LE(p.x, 10.0 + 1e-6);
    EXPECT_GE(p.y, -1e-6);
    EXPECT_LE(p.y, 10.0 + 1e-6);
  }
}

TEST(BoundaryPathTest, SingleSquareObstacleWithOffset) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(10, 0), Node2D(10, 10),
                        Node2D(0, 10)};
  BoundaryPathOptions opts{1.0, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GE(path.size(), 4u);

  // offset outward from obstacle: all path points should be outside the
  // obstacle, at distance >= offset from each edge
  // Original obstacle is [0,10]x[0,10]; offset path is in [-1,11]x[-1,11]
  for (const auto& p : path) {
    EXPECT_GE(p.x, -1.0 - 1e-4);
    EXPECT_LE(p.x, 11.0 + 1e-4);
    EXPECT_GE(p.y, -1.0 - 1e-4);
    EXPECT_LE(p.y, 11.0 + 1e-4);
  }
}

TEST(BoundaryPathTest, SingleSquareObstacleWithArc) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(10, 0), Node2D(10, 10),
                        Node2D(0, 10)};
  BoundaryPathOptions opts{1.0, 0.3, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GT(path.size(), 4u);  // arc adds extra points
}

TEST(BoundaryPathTest, SingleTriangleObstacle) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(6, 0), Node2D(3, 5)};
  BoundaryPathOptions opts{0.5, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GE(path.size(), 3u);
}

TEST(BoundaryPathTest, MultipleObstacles) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(15, -5), Node2D(15, 15),
                        Node2D(-5, 15)};
  std::vector<Polygon2D> obstacles = {
      {Node2D(0, 0), Node2D(3, 0), Node2D(3, 3), Node2D(0, 3)},
      {Node2D(8, 8), Node2D(12, 8), Node2D(12, 12), Node2D(8, 12)}};
  BoundaryPathOptions opts{0.5, 0.2, true, 8};

  auto paths = BoundaryPathGenerator::generateObstacleBoundaries(
      boundary, obstacles, opts);
  EXPECT_EQ(paths.size(), 2u);
  for (const auto& p : paths) {
    EXPECT_GT(p.size(), 0u);
  }
}

TEST(BoundaryPathTest, EmptyObstacles) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(15, -5), Node2D(15, 15),
                        Node2D(-5, 15)};
  std::vector<Polygon2D> obstacles;
  BoundaryPathOptions opts{0.5, 0.0, true, 8};

  auto paths = BoundaryPathGenerator::generateObstacleBoundaries(
      boundary, obstacles, opts);
  EXPECT_TRUE(paths.empty());
}

TEST(BoundaryPathTest, CounterClockwiseObstacle) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(0, 10), Node2D(10, 10),
                        Node2D(10, 0)};
  BoundaryPathOptions opts{0.0, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GE(path.size(), 4u);
}

TEST(BoundaryPathTest, DegenerateObstacle) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(0, 0), Node2D(1, 0),
                        Node2D(1, 1), Node2D(0, 1)};
  BoundaryPathOptions opts{0.0, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  EXPECT_GE(path.size(), 3u);
}

TEST(BoundaryPathTest, PathOrientationsValid) {
  Polygon2D obstacle = {Node2D(0, 0), Node2D(10, 0), Node2D(10, 10),
                        Node2D(0, 10)};
  BoundaryPathOptions opts{0.5, 0.0, true, 8};

  auto path =
      BoundaryPathGenerator::traceSingleObstacleBoundary(obstacle, opts);
  for (const auto& p : path) {
    EXPECT_GE(p.theta, -M_PI - 1e-6);
    EXPECT_LE(p.theta, M_PI + 1e-6);
  }
}

TEST(BoundaryPathTest, GlobalPathSingleObstacle) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(15, -5), Node2D(15, 15),
                        Node2D(-5, 15)};
  std::vector<Polygon2D> obstacles = {
      {Node2D(0, 0), Node2D(5, 0), Node2D(5, 5), Node2D(0, 5)}};
  BoundaryPathOptions opts{0.3, 0.0, true, 8};

  auto path = BoundaryPathGenerator::generateGlobalBoundaryPath(
      boundary, obstacles, opts);
  EXPECT_FALSE(path.poses.empty());
}

TEST(BoundaryPathTest, GlobalPathMultipleObstacles) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(20, -5), Node2D(20, 20),
                        Node2D(-5, 20)};
  std::vector<Polygon2D> obstacles = {
      {Node2D(0, 0), Node2D(3, 0), Node2D(3, 3), Node2D(0, 3)},
      {Node2D(10, 10), Node2D(14, 10), Node2D(14, 14), Node2D(10, 14)},
      {Node2D(0, 12), Node2D(4, 12), Node2D(4, 16), Node2D(0, 16)}};
  BoundaryPathOptions opts{0.5, 0.0, true, 8};

  auto path = BoundaryPathGenerator::generateGlobalBoundaryPath(
      boundary, obstacles, opts);
  EXPECT_FALSE(path.poses.empty());
  // should have at least all three obstacles worth of points
  EXPECT_GT(path.poses.size(), 3u * 4u);
}

TEST(BoundaryPathTest, GlobalPathEmptyObstacles) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(15, -5), Node2D(15, 15),
                        Node2D(-5, 15)};
  std::vector<Polygon2D> obstacles;
  BoundaryPathOptions opts{0.5, 0.0, true, 8};

  auto path = BoundaryPathGenerator::generateGlobalBoundaryPath(
      boundary, obstacles, opts);
  EXPECT_TRUE(path.poses.empty());
}

TEST(BoundaryPathTest, GlobalPathWithStartPose) {
  Polygon2D boundary = {Node2D(-10, -10), Node2D(20, -10), Node2D(20, 20),
                        Node2D(-10, 20)};
  std::vector<Polygon2D> obstacles = {
      {Node2D(0, 0), Node2D(4, 0), Node2D(4, 4), Node2D(0, 4)},
      {Node2D(10, 10), Node2D(14, 10), Node2D(14, 14), Node2D(10, 14)}};
  BoundaryPathOptions opts{0.3, 0.0, true, 8};

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = -5.0;
  start_pose.position.y = -5.0;
  start_pose.orientation.w = 1.0;

  auto path = BoundaryPathGenerator::generateGlobalBoundaryPath(
      boundary, obstacles, opts, start_pose);
  EXPECT_FALSE(path.poses.empty());
  EXPECT_GT(path.poses.size(), 2u * 4u);
}

TEST(BoundaryPathTest, GlobalPathWithArcSmoothing) {
  Polygon2D boundary = {Node2D(-5, -5), Node2D(15, -5), Node2D(15, 15),
                        Node2D(-5, 15)};
  std::vector<Polygon2D> obstacles = {
      {Node2D(0, 0), Node2D(4, 0), Node2D(4, 4), Node2D(0, 4)},
      {Node2D(8, 8), Node2D(12, 8), Node2D(12, 12), Node2D(8, 12)}};
  BoundaryPathOptions opts{0.5, 0.3, true, 8};

  auto path = BoundaryPathGenerator::generateGlobalBoundaryPath(
      boundary, obstacles, opts);
  EXPECT_FALSE(path.poses.empty());
  // arc smoothing adds extra points
  EXPECT_GT(path.poses.size(), 2u * 4u);
}
