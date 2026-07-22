#include <gtest/gtest.h>

#include <memory>
#include <nav_msgs/msg/path.hpp>

#include "common/node2d.h"
#include "map/static_map.h"
#include "planner/global_planner/a_star.h"
#include "planner/global_planner/global_planner_base.h"

using namespace common;
using namespace global_planner;
using namespace map;

namespace {

// 用于测试的 dummy planner：直接返回 start->goal 的直线路径
class DummyPlanner : public GlobalPlannerBase {
 public:
  DummyPlanner(std::shared_ptr<StaticMap> map, const rclcpp::Logger logger)
      : GlobalPlannerBase(map, logger) {}

  nav_msgs::msg::Path searchPath() override {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped p;
    p.pose = start_pose_;
    path.poses.push_back(p);
    p.pose = goal_pose_;
    path.poses.push_back(p);
    return path;
  }
};

}  // namespace

TEST(GlobalPlannerBaseTest, CompleteCoverPathDummyPlanner) {
  rclcpp::Logger logger = rclcpp::get_logger("test_global_planner_base");
  auto static_map = std::make_shared<StaticMap>();

  DummyPlanner planner(static_map, logger);

  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = -4.2;
  start_pose.position.y = -4.2;
  start_pose.orientation.w = 1.0;
  planner.setStartPose(start_pose);

  Polygon2D boundary = {Node2D(-4.0, -4.0), Node2D(-1.0, -4.0),
                        Node2D(-1.0, -1.0), Node2D(-4.0, -1.0)};
  planner.setCoverArea(boundary, {});
  planner.setSweepParams(0.6, Node2D(1.0, 0.0));

  nav_msgs::msg::Path cover_path = planner.getGlobalCoverPath();
  EXPECT_FALSE(cover_path.poses.empty());

  nav_msgs::msg::Path complete_path = planner.generateCompleteCoverPath();
  EXPECT_FALSE(complete_path.poses.empty());

  // 完整路径应至少包含 approach 的 2 个点 + coverage 路径（连接点去重）
  EXPECT_GT(complete_path.poses.size(), cover_path.poses.size());

  std::cout << "Cover path size: " << cover_path.poses.size() << std::endl;
  std::cout << "Complete path size: " << complete_path.poses.size()
            << std::endl;
}

TEST(GlobalPlannerBaseTest, CompleteCoverPathWithAStar) {
  rclcpp::Logger logger = rclcpp::get_logger("test_global_planner_base");
  auto static_map = std::make_shared<StaticMap>();

  AStar planner(static_map, logger);

  // 自车位置在 coverage 区域左下方
  geometry_msgs::msg::Pose start_pose;
  start_pose.position.x = -4.5;
  start_pose.position.y = -4.5;
  start_pose.orientation.w = 1.0;
  planner.setStartPose(start_pose);

  // 覆盖区域在地图左下角，避开默认障碍物
  Polygon2D boundary = {Node2D(-5.0, -5.0), Node2D(5.0, -5.0), Node2D(5.0, 5.0),
                        Node2D(-5.0, 5.0)};
  std::vector<Polygon2D> obstacles;
  Polygon2D obstacle = {Node2D(-1.0, -1.0), Node2D(1.0, -1.0), Node2D(1.0, 1.0),
                        Node2D(-1.0, 1.0)};
  obstacles.emplace_back(obstacle);
  obstacle = {Node2D(2.0, 1.0), Node2D(3.0, 2.0), Node2D(3.0, 2.0)};
  obstacles.emplace_back(obstacle);
  planner.setCoverArea(boundary, obstacles);
  planner.setSweepParams(0.6, Node2D(1.0, 0.0));

  nav_msgs::msg::Path cover_path = planner.getGlobalCoverPath();
  EXPECT_FALSE(cover_path.poses.empty());

  nav_msgs::msg::Path complete_path = planner.generateCompleteCoverPath();
  EXPECT_FALSE(complete_path.poses.empty());

  // 使用 A* 时应能规划出 approach 路径，因此完整路径比纯 coverage 路径长
  EXPECT_GT(complete_path.poses.size(), cover_path.poses.size());

  auto& poses = complete_path.poses;
  for (const auto& node : poses) {
    std::cout << "x: " << node.pose.position.x << " y: " << node.pose.position.y
              << std::endl;
  }

  std::cout << "AStar cover path size: " << cover_path.poses.size()
            << std::endl;
  std::cout << "AStar complete path size: " << complete_path.poses.size()
            << std::endl;
}
