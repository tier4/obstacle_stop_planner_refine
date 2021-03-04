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

#ifndef OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_
#define OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_

#include <algorithm>
#include <vector>
#include <memory>
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle_stop_planner/vehicle.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "obstacle_stop_planner/util/create_vehicle_footprint.hpp"

namespace obstacle_stop_planner
{

class OneStepPolygon
{
public:
  void create(
    const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
    const double expand_width);
  autoware_utils::Polygon2d getPolygon() const {return polygon_;}
  void setVehicleInfo(VehicleInfo & vehicle_info)
  {
    vehicle_info_ = std::make_shared<VehicleInfo>(vehicle_info);
  }

private:
  autoware_utils::Polygon2d polygon_;
  std::shared_ptr<VehicleInfo> vehicle_info_;
};

inline void OneStepPolygon::create(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const double expand_width)
{
  autoware_utils::Polygon2d one_step_move_vehicle_corner_points;
  const auto footprint = createVehicleFootprint(*vehicle_info_, 0.0, expand_width);

  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_step_pose.orientation);

    bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> rotate(yaw);
    autoware_utils::LinearRing2d transformed_footprint;
    bg::transform(footprint, transformed_footprint, rotate);

    bg::strategy::transform::translate_transformer<double, 2, 2> translate(
      base_step_pose.position.x, base_step_pose.position.y);
    bg::transform(transformed_footprint, transformed_footprint, translate);
    one_step_move_vehicle_corner_points.outer() = transformed_footprint;
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> rotate(yaw);
    autoware_utils::LinearRing2d transformed_footprint;
    bg::transform(footprint, transformed_footprint, rotate);

    bg::strategy::transform::translate_transformer<double, 2, 2> translate(
      base_step_pose.position.x, base_step_pose.position.y);
    bg::transform(transformed_footprint, transformed_footprint, translate);
    for (const auto & item : transformed_footprint) {
      one_step_move_vehicle_corner_points.outer().emplace_back(item);
    }
  }

  bg::convex_hull(one_step_move_vehicle_corner_points, polygon_);
}

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_
