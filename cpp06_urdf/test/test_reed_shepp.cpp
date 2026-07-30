#include <gtest/gtest.h>

#include <cmath>

#include "planner/global_planner/reed_shepp.h"

using namespace global_planner;

TEST(ReedsSheppTest, StraightLine) {
  double q0[] = {0.0, 0.0, 0.0};
  double q1[] = {10.0, 0.0, 0.0};
  ReedsSheppPath path;
  EXPECT_EQ(reeds_shepp_init(q0, q1, 5.0, &path), 0);
  EXPECT_GT(reeds_shepp_path_length(&path), 0.0);

  double q[3];
  EXPECT_EQ(reeds_shepp_path_sample(&path, reeds_shepp_path_length(&path), q), 0);
  EXPECT_NEAR(q[0], q1[0], 1e-6);
  EXPECT_NEAR(q[1], q1[1], 1e-6);
  EXPECT_NEAR(q[2], q1[2], 1e-6);
}

TEST(ReedsSheppTest, TurnInPlace) {
  double q0[] = {0.0, 0.0, 0.0};
  double q1[] = {0.0, 0.0, M_PI};
  ReedsSheppPath path;
  EXPECT_EQ(reeds_shepp_init(q0, q1, 5.0, &path), 0);
  EXPECT_GT(reeds_shepp_path_length(&path), 0.0);

  double q[3];
  EXPECT_EQ(reeds_shepp_path_sample(&path, reeds_shepp_path_length(&path), q), 0);
  EXPECT_NEAR(q[0], q1[0], 1e-6);
  EXPECT_NEAR(q[1], q1[1], 1e-6);
  EXPECT_NEAR(q[2], q1[2], 1e-6);
}

TEST(ReedsSheppTest, SampleMidPoint) {
  double q0[] = {0.0, 0.0, 0.0};
  double q1[] = {10.0, 0.0, 0.0};
  ReedsSheppPath path;
  EXPECT_EQ(reeds_shepp_init(q0, q1, 5.0, &path), 0);

  double total_length = reeds_shepp_path_length(&path);
  double q[3];
  EXPECT_EQ(reeds_shepp_path_sample(&path, total_length * 0.5, q), 0);
  EXPECT_NEAR(q[1], 0.0, 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
