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

#ifndef OBSTACLE_STOP_PLANNER__PARAMETER__STOP_CONTROL_PARAMETER_HPP_
#define OBSTACLE_STOP_PLANNER__PARAMETER__STOP_CONTROL_PARAMETER_HPP_

#include <obstacle_stop_planner/visibility_control.hpp>

namespace obstacle_stop_planner
{
struct OBSTACLE_STOP_PLANNER_PUBLIC StopControlParameter
{
  double stop_margin;
  double min_behavior_stop_margin;
  double step_length;
  double extend_distance;
  double expand_stop_range;
  double stop_search_radius;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__PARAMETER__STOP_CONTROL_PARAMETER_HPP_
