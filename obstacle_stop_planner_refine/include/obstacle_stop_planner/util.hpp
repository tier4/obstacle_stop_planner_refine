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

#include "opencv2/core/core.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "boost/geometry.hpp"
#include "boost/format.hpp"
#include "boost/assign/list_of.hpp"
#include "tf2/utils.h"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "autoware_utils/autoware_utils.hpp"

namespace
{
inline double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

  return yaw;
}

inline std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
      R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
    pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
    pose.orientation.y % pose.orientation.z)
    .str();
  return json_dumps_pose;
}

inline diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}

inline cv::Point2d calcCentroid(const std::vector<cv::Point2d> & pointcloud)
{
  cv::Point2d centroid;
  centroid.x = 0;
  centroid.y = 0;
  for (const auto & point : pointcloud) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(pointcloud.size());
  centroid.y = centroid.y / static_cast<double>(pointcloud.size());
  return centroid;
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

inline autoware_utils::Polygon2d getPolygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size,
  const double center_offset, const double l_margin = 0.0, const double w_margin = 0.0)
{
  autoware_utils::Polygon2d obj_poly;
  geometry_msgs::msg::Vector3 obj_rpy = rpyFromQuat(pose.orientation);

  double l = size.x * std::cos(obj_rpy.y) + l_margin;
  double w = size.y * std::cos(obj_rpy.x) + w_margin;
  double co = center_offset;
  boost::geometry::exterior_ring(obj_poly) =
    boost::assign::list_of<autoware_utils::Point2d>(l / 2.0 + co, w / 2.0)(-l / 2.0 + co, w / 2.0)(
    -l / 2.0 + co, -w / 2.0)(l / 2.0 + co, -w / 2.0)(l / 2.0 + co, w / 2.0);

  // rotate polygon
  // original:clockwise
  boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::radian, double, 2, 2> rotate(-obj_rpy.z);
  // rotation
  autoware_utils::Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);
  // translate polygon
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(
    pose.position.x,
    pose.position.y);
  autoware_utils::Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

inline double getDistanceFromTwoPoint(
  const geometry_msgs::msg::Point & point1,
  const geometry_msgs::msg::Point & point2)
{
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  const double dist = std::hypot(dx, dy);
  return dist;
}
}  // namespace

#endif  // OBSTACLE_STOP_PLANNER__UTIL_HPP_
