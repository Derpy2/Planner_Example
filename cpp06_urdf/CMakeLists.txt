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
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(include)
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(global_planner_node
  src/map/static_map.cpp
  src/map/common/kd_tree.cpp
  src/planner/global_planner/a_star.cpp
  src/planner/global_planner/hybrid_a_star.cpp
  src/planner/global_planner/dubins.cpp
  src/planner/global_planner/reed_shepp.cpp
  src/planner/global_planner/rrt_star.cpp
  src/planner/global_planner/complete_cover_path/bcd.cpp
  src/planner/global_planner/complete_cover_path/sweep.cpp
  src/planner/global_planner/global_planner_base.cpp
  src/common/node3d.cpp
  src/common/collision_detection.cpp
  src/global_planner_node.cpp
  src/visualization/visualization_manager.cpp
  include/global_planner_node.h)
ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(global_planner_node ${OMPL_LIBRARIES})

add_executable(local_planner_node
  src/map/static_map.cpp
  src/map/common/kd_tree.cpp
  src/local_planner_node.cpp
  include/local_planner_node.h
  src/planner/reference_line/slide_window.cpp
  src/planner/reference_line/b_spline.cpp
  src/planner/reference_line/bezier.cpp
  src/planner/local_planner/pure_pursuit.cpp
  src/planner/local_planner/mpc.cpp
  src/planner/local_planner/dwa.cpp
  src/planner/local_planner/teb.cpp
  src/visualization/visualization_manager.cpp
)
ament_target_dependencies(local_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  Eigen3
  visualization_msgs
)

target_link_libraries(local_planner_node osqp::osqp OsqpEigen::OsqpEigen)

install(TARGETS
  global_planner_node
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf rviz meshes msg
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hybrid_a_star
    test/test_hybrid_a_star.cpp
    src/map/static_map.cpp
    src/map/common/kd_tree.cpp
    src/planner/global_planner/hybrid_a_star.cpp
    src/planner/global_planner/dubins.cpp
    src/planner/global_planner/reed_shepp.cpp
    src/common/node3d.cpp
    src/common/collision_detection.cpp
    src/planner/global_planner/a_star.cpp
    src/visualization/visualization_manager.cpp
  )
  target_include_directories(test_hybrid_a_star PUBLIC include)
  target_include_directories(test_hybrid_a_star PUBLIC ${OMPL_INCLUDE_DIRS})
  ament_target_dependencies(test_hybrid_a_star
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
  )
  target_link_libraries(test_hybrid_a_star ${OMPL_LIBRARIES})

  ament_add_gtest(test_reed_shepp
    test/test_reed_shepp.cpp
    src/planner/global_planner/reed_shepp.cpp
  )
  target_include_directories(test_reed_shepp PUBLIC include)
  target_include_directories(test_reed_shepp PUBLIC ${OMPL_INCLUDE_DIRS})
  ament_target_dependencies(test_reed_shepp
    rclcpp
  )
  target_link_libraries(test_reed_shepp ${OMPL_LIBRARIES})

  ament_add_gtest(test_dwa
    test/test_dwa.cpp
    src/planner/local_planner/dwa.cpp
    src/common/node3d.cpp
    src/common/collision_detection.cpp
    src/map/static_map.cpp
    src/map/common/kd_tree.cpp
    src/visualization/visualization_manager.cpp
  )
  target_include_directories(test_dwa PUBLIC include)
  ament_target_dependencies(test_dwa
    rclcpp
    nav_msgs
    geometry_msgs
    visualization_msgs
  )

  ament_add_gtest(test_bcd
    test/test_bcd.cpp
    src/planner/global_planner/complete_cover_path/bcd.cpp
    src/planner/global_planner/complete_cover_path/sweep.cpp
  )
  target_include_directories(test_bcd PUBLIC include)
  ament_target_dependencies(test_bcd
    rclcpp
  )

  ament_add_gtest(test_global_planner_base
    test/test_global_planner_base.cpp
    src/planner/global_planner/global_planner_base.cpp
    src/planner/global_planner/complete_cover_path/bcd.cpp
    src/planner/global_planner/complete_cover_path/sweep.cpp
    src/planner/global_planner/a_star.cpp
    src/map/static_map.cpp
    src/map/common/kd_tree.cpp
  )
  target_include_directories(test_global_planner_base PUBLIC include)
  ament_target_dependencies(test_global_planner_base
    rclcpp
    nav_msgs
  )
endif()

ament_package()
