cmake_minimum_required(VERSION 3.5)
project(obstacle_stop_planner_refine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE OBSTACLE_STOP_PLANNER_NODES_NODE_SRC
  src/*
)
file(GLOB_RECURSE OBSTACLE_STOP_PLANNER_NODES_NODE_HEADERS
  include/obstacle_stop_planner/*
)

# generate component node library
ament_auto_add_library(obstacle_stop_planner_node SHARED
  ${OBSTACLE_STOP_PLANNER_NODES_NODE_SRC}
  ${OBSTACLE_STOP_PLANNER_NODES_NODE_HEADERS}
)
rclcpp_components_register_node(obstacle_stop_planner_node
  PLUGIN "obstacle_stop_planner::ObstacleStopPlannerNode"
  EXECUTABLE obstacle_stop_planner_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/obstacle_stop_planner_node_launch_test.py
    TIMEOUT "30"
  )

  find_package(ament_cmake_auto_gtest REQUIRED)
  ament_auto_add_gtest(test_obstacle_stop_planner
    DIRECTORY "test/"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
