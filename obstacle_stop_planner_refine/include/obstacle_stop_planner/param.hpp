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

#ifndef OBSTACLE_STOP_PLANNER__PARAM_HPP_
#define OBSTACLE_STOP_PLANNER__PARAM_HPP_

#include "autoware_utils/autoware_utils.hpp"

namespace obstacle_stop_planner
{

struct Param
{
  VehicleInfo vehicle_info;
  double stop_margin;
  double min_behavior_stop_margin;
  double step_length;
  double extend_distance;
  double expand_stop_range;
  double slow_down_margin;
  double expand_slow_down_range;
  double max_slow_down_vel;
  double min_slow_down_vel;
  double max_deceleration;
  bool enable_slow_down;
  double stop_search_radius;
  double slow_down_search_radius;
};

inline double getSearchRadius(const Param & param)
{
  if (param.enable_slow_down) {
    return param.slow_down_search_radius;
  } else {
    return param.stop_search_radius;
  }
}

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__PARAM_HPP_
