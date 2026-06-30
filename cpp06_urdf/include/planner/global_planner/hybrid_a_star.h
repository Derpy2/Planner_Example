#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/collision_detection.h"
#include "common/node3d.h"
#include "dubins.h"
#include "global_planner_base.h"

typedef ompl::base::SE2StateSpace::StateType State;

namespace global_planner {

using common::Node3D;

class HybridAStar : public GlobalPlannerBase {
 public:
  HybridAStar(std::shared_ptr<map::StaticMap> map, const rclcpp::Logger& logger,
              const common::CollisionDetection& cs)
      : GlobalPlannerBase(map, logger), configuration_space_(cs) {}

  nav_msgs::msg::Path searchPath() override;

  // start and goal is grid position
  nav_msgs::msg::Path searchPath(Node3D& start, const Node3D& goal);

  void updateH(Node3D& start, const Node3D& goal);

  std::shared_ptr<Node3D> dubinsShot(
      const std::shared_ptr<Node3D>& start, const Node3D& goal,
      common::CollisionDetection& configuration_space);

 private:
  bool enable_reverse = false;
  common::CollisionDetection configuration_space_;
  std::vector<std::shared_ptr<Node3D>> node3d_;
};

}  // namespace global_planner