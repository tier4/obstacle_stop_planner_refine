// Copyright 2019 Autoware Foundation
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
#ifndef OBSTACLE_STOP_PLANNER__UTIL_HPP_
#define OBSTACLE_STOP_PLANNER__UTIL_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "boost/geometry.hpp"
#include "boost/format.hpp"
#include "boost/assign/list_of.hpp"
#include "tf2/utils.h"
#include "autoware_utils/autoware_utils.hpp"

using autoware_utils::Point2d;
using autoware_utils::Point3d;
using autoware_utils::Polygon2d;
using autoware_utils::Polygon3d;

namespace
{
inline double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  return yaw;
}

inline geometry_msgs::msg::Pose getVehicleCenterFromBase(
  const geometry_msgs::msg::Pose & base_pose,
  const double vehicle_length,
  const double rear_overhang)
{
  geometry_msgs::msg::Pose center_pose;
  const double yaw = getYawFromQuaternion(base_pose.orientation);
  center_pose.position.x =
    base_pose.position.x + (vehicle_length / 2.0 - rear_overhang) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (vehicle_length / 2.0 - rear_overhang) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}

inline geometry_msgs::msg::Vector3 rpyFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}

inline Polygon2d getPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size,
  const double center_offset, const double l_margin = 0.0, const double w_margin = 0.0)
{
  Polygon2d obj_poly;
  geometry_msgs::msg::Vector3 obj_rpy = rpyFromQuat(pose.orientation);

  double l = size.x * std::cos(obj_rpy.y) + l_margin;
  double w = size.y * std::cos(obj_rpy.x) + w_margin;
  double co = center_offset;
  boost::geometry::exterior_ring(obj_poly) =
    boost::assign::list_of<Point2d>(l / 2.0 + co, w / 2.0)(-l / 2.0 + co, w / 2.0)(
    -l / 2.0 + co, -w / 2.0)(l / 2.0 + co, -w / 2.0)(l / 2.0 + co, w / 2.0);

  // rotate polygon
  // original:clockwise
  boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::radian, double, 2, 2> rotate(-obj_rpy.z);
  // rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);
  // translate polygon
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(
    pose.position.x,
    pose.position.y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}
}  // namespace

#endif  // OBSTACLE_STOP_PLANNER__UTIL_HPP_
