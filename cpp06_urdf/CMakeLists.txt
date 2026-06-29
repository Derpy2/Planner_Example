cmake_minimum_required(VERSION 3.8)
project(cpp06_urdf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

include_directories(include)
include_directories(/usr/include/ompl-1.5)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(global_planner_node
  src/map/static_map.cpp
  src/planner/global_planner/a_star.cpp
  src/global_planner_node.cpp
  include/global_planner_node.h)
ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ompl
)

add_executable(local_planner_node
  src/local_planner_node.cpp
  include/local_planner_node.h
  include/planner/reference_line/reference_line_factory.h
  include/planner/reference_line/reference_line_base.h
  src/planner/reference_line/slide_window.cpp
  src/planner/reference_line/b_spline.cpp
)
ament_target_dependencies(local_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS
  global_planner_node
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
