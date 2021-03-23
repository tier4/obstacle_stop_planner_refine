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

project(obstacle_stop_planner_refine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
  add_compile_options(-Wno-unused-parameter)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(obstacle_stop_planner SHARED
  src/debug_marker.cpp
  src/obstacle_stop_planner.cpp
  src/obstacle_point_cloud.cpp
  src/point_helper.cpp
  src/trajectory.cpp
  src/controller/adaptive_cruise_control.cpp
  src/controller/slow_down_control.cpp
  src/controller/stop_control.cpp
)

target_include_directories(obstacle_stop_planner
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(obstacle_stop_planner
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  include(test/cmake/ament_cmake_auto_gtest-extras.cmake)
  ament_auto_add_gtest(test_obstacle_stop_planner
    DIRECTORY "test"
  )
  target_include_directories(test_obstacle_stop_planner
    PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(test_obstacle_stop_planner
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
)
