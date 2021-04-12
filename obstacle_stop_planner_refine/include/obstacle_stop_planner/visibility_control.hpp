// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OBSTACLE_STOP_PLANNER__VISIBILITY_CONTROL_HPP_
#define OBSTACLE_STOP_PLANNER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OBSTACLE_STOP_PLANNER_EXPORT __attribute__ ((dllexport))
    #define OBSTACLE_STOP_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define OBSTACLE_STOP_PLANNER_EXPORT __declspec(dllexport)
    #define OBSTACLE_STOP_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef OBSTACLE_STOP_PLANNER_BUILDING_LIBRARY
    #define OBSTACLE_STOP_PLANNER_PUBLIC OBSTACLE_STOP_PLANNER_EXPORT
  #else
    #define OBSTACLE_STOP_PLANNER_PUBLIC OBSTACLE_STOP_PLANNER_IMPORT
  #endif
  #define OBSTACLE_STOP_PLANNER_PUBLIC_TYPE OBSTACLE_STOP_PLANNER_PUBLIC
  #define OBSTACLE_STOP_PLANNER_LOCAL
#else
  #define OBSTACLE_STOP_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define OBSTACLE_STOP_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define OBSTACLE_STOP_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define OBSTACLE_STOP_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OBSTACLE_STOP_PLANNER_PUBLIC
    #define OBSTACLE_STOP_PLANNER_LOCAL
  #endif
  #define OBSTACLE_STOP_PLANNER_PUBLIC_TYPE
#endif

#endif  // OBSTACLE_STOP_PLANNER__VISIBILITY_CONTROL_HPP_
