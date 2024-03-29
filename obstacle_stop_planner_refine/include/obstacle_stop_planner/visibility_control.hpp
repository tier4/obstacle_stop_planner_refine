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

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(OBSTACLE_STOP_PLANNER_BUILDING_DLL) || defined(OBSTACLE_STOP_PLANNER_EXPORTS)
    #define OBSTACLE_STOP_PLANNER_PUBLIC __declspec(dllexport)
    #define OBSTACLE_STOP_PLANNER_LOCAL
  #else
    #define OBSTACLE_STOP_PLANNER_PUBLIC __declspec(dllimport)
    #define OBSTACLE_STOP_PLANNER_LOCAL
  #endif
#elif defined(__linux__)
  #define OBSTACLE_STOP_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define OBSTACLE_STOP_PLANNER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define OBSTACLE_STOP_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define OBSTACLE_STOP_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // OBSTACLE_STOP_PLANNER__VISIBILITY_CONTROL_HPP_
