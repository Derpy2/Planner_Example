#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "map/static_map.h"
#include "planner/local_planner/dwa.h"

using namespace local_planner;

TEST(DWATest, ReverseSegmentProducesNegativeVelocity) {
  rclcpp::Logger logger = rclcpp::get_logger("test_dwa");
  auto static_map = std::make_shared<map::StaticMap>();
  LocalPlannerDWA dwa(static_map, logger);
  dwa.init();

  // Robot at origin, facing +x.
  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  dwa.setCurrentPose(pose);

  geometry_msgs::msg::TwistWithCovariance twist;
  twist.twist.linear.x = 0.0;
  twist.twist.angular.z = 0.0;
  dwa.setCurrentTwist(twist);

  // Reference line: vehicle faces +x but the path goes toward -x (reverse).
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.pose.orientation.w = 1.0;
  path.poses.push_back(p);

  p.pose.position.x = -1.0;
  p.pose.position.y = 0.0;
  p.pose.orientation.w = 1.0;
  path.poses.push_back(p);

  dwa.setSmoothedPath(path);
  dwa.setNearsetIdx(0);

  geometry_msgs::msg::Twist cmd = dwa.getControlCmd();

  // With reverse support enabled, DWA should command a negative linear velocity.
  EXPECT_LT(cmd.linear.x, 0.0)
      << "DWA did not command reverse velocity for a reverse segment.";
}

TEST(DWATest, ForwardSegmentProducesPositiveVelocity) {
  rclcpp::Logger logger = rclcpp::get_logger("test_dwa");
  auto static_map = std::make_shared<map::StaticMap>();
  LocalPlannerDWA dwa(static_map, logger);
  dwa.init();

  geometry_msgs::msg::PoseWithCovariance pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  dwa.setCurrentPose(pose);

  geometry_msgs::msg::TwistWithCovariance twist;
  twist.twist.linear.x = 0.0;
  twist.twist.angular.z = 0.0;
  dwa.setCurrentTwist(twist);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.pose.orientation.w = 1.0;
  path.poses.push_back(p);

  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.pose.orientation.w = 1.0;
  path.poses.push_back(p);

  dwa.setSmoothedPath(path);
  dwa.setNearsetIdx(0);

  geometry_msgs::msg::Twist cmd = dwa.getControlCmd();

  EXPECT_GT(cmd.linear.x, 0.0)
      << "DWA did not command forward velocity for a forward segment.";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
