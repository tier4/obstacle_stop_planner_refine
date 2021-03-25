# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.5)

project(obstacle_stop_planner_nodes)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-parameter)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(OBSTACLE_STOP_PLANNER_NODES_NODE_SRC
  src/obstacle_stop_planner_node.cpp
)

set(OBSTACLE_STOP_PLANNER_NODES_NODE_HEADERS
  include/obstacle_stop_planner_nodes/obstacle_stop_planner_node.hpp
)

# generate component node library
ament_auto_add_library(obstacle_stop_planner_node SHARED
  ${OBSTACLE_STOP_PLANNER_NODES_NODE_SRC}
  ${OBSTACLE_STOP_PLANNER_NODES_NODE_HEADERS}
)

rclcpp_components_register_node(obstacle_stop_planner_node
  PLUGIN "autoware::obstacle_stop_planner_nodes::ObstacleStopPlannerNode"
  EXECUTABLE obstacle_stop_planner_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/obstacle_stop_planner_node_launch_test.py
    TIMEOUT "30"
  )

  # Unit tests
  set(TEST_SOURCES test/test_obstacle_stop_planner_node.cpp)
  set(TEST_OBSTACLE_STOP_PLANNER_NODE_EXE test_obstacle_stop_planner_node)
  ament_add_gtest(${TEST_OBSTACLE_STOP_PLANNER_NODE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_OBSTACLE_STOP_PLANNER_NODE_EXE} obstacle_stop_planner_node)
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
