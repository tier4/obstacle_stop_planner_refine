// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef OBSTACLE_STOP_PLANNER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
#define OBSTACLE_STOP_PLANNER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_

#include "autoware_utils/autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

namespace obstacle_stop_planner
{
inline autoware_utils::LinearRing2d createVehicleFootprint(
  const vehicle_info_util::VehicleInfo & vehicle_info, const double top_margin = 0.0,
  const double side_margin = 0.0)
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m_ + i.wheel_base_m_ + top_margin;
  const double x_rear = -(i.rear_overhang_m_ + top_margin);
  const double y_left = i.wheel_tread_m_ / 2.0 + i.left_overhang_m_ + side_margin;
  const double y_right = -(i.wheel_tread_m_ / 2.0 + i.right_overhang_m_ + side_margin);

  LinearRing2d footprint;
  footprint.push_back(Point2d{x_front, y_left});
  footprint.push_back(Point2d{x_front, y_right});
  footprint.push_back(Point2d{x_rear, y_right});
  footprint.push_back(Point2d{x_rear, y_left});
  footprint.push_back(Point2d{x_front, y_left});

  return footprint;
}
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
