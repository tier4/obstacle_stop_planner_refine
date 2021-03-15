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

#ifndef OBSTACLE_STOP_PLANNER__UTIL__ONE_STEP_POLYGON_HPP_
#define OBSTACLE_STOP_PLANNER__UTIL__ONE_STEP_POLYGON_HPP_

#include <algorithm>
#include <vector>
#include <memory>
#include "boost/geometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle_stop_planner/util/create_vehicle_footprint.hpp"
#include "obstacle_stop_planner/util/util.hpp"

namespace obstacle_stop_planner
{
inline Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const double expand_width, const VehicleInfo & vehicle_info)
{
  Polygon2d one_step_move_vehicle_corner_points;
  const auto footprint = createVehicleFootprint(vehicle_info, 0.0, expand_width);

  // start step
  {
    double yaw = getYawFromQuaternion(base_step_pose.orientation);

    boost::geometry::strategy::transform::rotate_transformer<
      boost::geometry::radian, double, 2, 2> rotate(yaw);
    autoware_utils::LinearRing2d transformed_footprint;
    boost::geometry::transform(footprint, transformed_footprint, rotate);

    boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(
      base_step_pose.position.x, base_step_pose.position.y);
    boost::geometry::transform(transformed_footprint, transformed_footprint, translate);
    one_step_move_vehicle_corner_points.outer() = transformed_footprint;
  }
  // next step
  {
    double yaw = getYawFromQuaternion(next_step_pose.orientation);
    boost::geometry::strategy::transform::rotate_transformer<
      boost::geometry::radian, double, 2, 2> rotate(yaw);
    autoware_utils::LinearRing2d transformed_footprint;
    boost::geometry::transform(footprint, transformed_footprint, rotate);

    boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(
      base_step_pose.position.x, base_step_pose.position.y);
    boost::geometry::transform(transformed_footprint, transformed_footprint, translate);
    for (const auto & item : transformed_footprint) {
      one_step_move_vehicle_corner_points.outer().emplace_back(item);
    }
  }

  Polygon2d polygon;
  boost::geometry::convex_hull(one_step_move_vehicle_corner_points, polygon);
  return polygon;
}

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__UTIL__ONE_STEP_POLYGON_HPP_
