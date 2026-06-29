#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "planner/global_planner/hybrid_a_star.h"

using namespace global_planner;
using namespace common;

class HybridAStarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    map_ = std::make_shared<map::StaticMap>();
    CollisionDetection cd(map_);
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("test_hybrid_a_star"));
    planner_ = std::make_unique<HybridAStar>(map_, *logger_, cd);
  }

  void TearDown() override {
    planner_.reset();
    map_.reset();
    logger_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<map::StaticMap> map_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<HybridAStar> planner_;
};

TEST_F(HybridAStarTest, Initialization) {
  ASSERT_NE(planner_, nullptr);
}

TEST_F(HybridAStarTest, RandomPathPlanning) {
  ASSERT_NE(planner_, nullptr);

  // Set a random start and goal pose in free space (world coordinates)
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = -3.0;
  start_pose.position.y = -3.0;
  start_pose.orientation.w = 1.0;

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = 3.0;
  goal_pose.position.y = -3.0;
  goal_pose.orientation.w = 1.0;

  planner_->setStartPose(start_pose);
  planner_->setGoalPose(goal_pose);

  nav_msgs::msg::Path path = planner_->searchPath();

  // The path should not be empty when a solution is found
  EXPECT_FALSE(path.poses.empty())
      << "Hybrid A* failed to find a path between start and goal.";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
