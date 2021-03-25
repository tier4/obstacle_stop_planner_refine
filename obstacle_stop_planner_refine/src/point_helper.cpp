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

#include <limits>
#include <algorithm>
#include <tuple>

#include "obstacle_stop_planner/util/point_helper.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace obstacle_stop_planner
{
Point2d pointXYZtoPoint2d(const pcl::PointXYZ & point)
{
  return Point2d(point.x, point.y);
}

Point2d diff(const pcl::PointXYZ & p1, const Point & p2)
{
  return Point2d(p1.x - p2.x, p1.y - p2.y);
}


Point2d PointHelper::getBackwardPointFromBasePoint(
  const Point2d & line_point1, const Point2d & line_point2,
  const Point2d & base_point, const double backward_length) const
{
  const auto line_vec = line_point2 - line_point1;
  const auto backward_vec = backward_length * line_vec.normalized();
  const auto output_point = base_point + backward_vec;
  return Point2d(output_point.x(), output_point.y());
}

PointStamped PointHelper::getNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const Pose & base_pose) const
{
  const double yaw = getYawFromQuaternion(base_pose.orientation);
  Point2d base_pose_vec {std::cos(yaw), std::sin(yaw)};

  const auto pointcloud_vec = diff(pointcloud.at(0), base_pose.position);
  double min_norm = base_pose_vec.dot(pointcloud_vec);
  PointStamped nearest_collision_point;
  nearest_collision_point.point = pointXYZtoPoint2d(pointcloud.at(0));
  nearest_collision_point.time = pcl_conversions::fromPCL(pointcloud.header).stamp;

  for (size_t i = 1; i < pointcloud.size(); ++i) {
    const auto pointcloud_vec = diff(pointcloud.at(i), base_pose.position);
    double norm = base_pose_vec.dot(pointcloud_vec);

    if (norm < min_norm) {
      min_norm = norm;
      nearest_collision_point.point = pointXYZtoPoint2d(pointcloud.at(i));
      nearest_collision_point.time = pcl_conversions::fromPCL(pointcloud.header).stamp;
    }
  }
  return nearest_collision_point;
}

PointDeviation PointHelper::getLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const Pose & base_pose) const
{
  double min_norm = std::numeric_limits<double>::max();
  const double yaw = getYawFromQuaternion(base_pose.orientation);
  const Point2d base_pose_vec {std::cos(yaw), std::sin(yaw)};
  PointDeviation lateral_nearest_point;

  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const auto pointcloud_vec = diff(pointcloud.at(i), base_pose.position);
    double norm =
      std::abs(base_pose_vec.x() * pointcloud_vec.y() - base_pose_vec.y() * pointcloud_vec.x());
    if (norm < min_norm) {
      min_norm = norm;
      lateral_nearest_point.point = pointcloud.at(i);
    }
  }
  lateral_nearest_point.deviation = min_norm;
  return lateral_nearest_point;
}

Trajectory PointHelper::insertStopPoint(
  const StopPoint & stop_point, const Trajectory & base_path,
  const Trajectory & input_path) const
{
  auto output_path = input_path;
  TrajectoryPoint stop_trajectory_point =
    base_path.points.at(std::max(static_cast<int>(stop_point.index) - 1, 0));
  stop_trajectory_point.pose.position.x = stop_point.point.x();
  stop_trajectory_point.pose.position.y = stop_point.point.y();
  stop_trajectory_point.twist.linear.x = 0.0;
  output_path.points.insert(output_path.points.begin() + stop_point.index, stop_trajectory_point);
  for (size_t j = stop_point.index; j < output_path.points.size(); ++j) {
    output_path.points.at(j).twist.linear.x = 0.0;
  }
  return output_path;
}

StopPoint PointHelper::searchInsertPoint(
  const int idx, const Trajectory & base_path,
  const Point2d & trajectory_vec,
  const Point2d & collision_point_vec,
  const StopControlParameter & param) const
{
  const auto max_dist_stop_point =
    createTargetPoint(
    idx, param.stop_margin, trajectory_vec, collision_point_vec,
    base_path);
  const auto min_dist_stop_point = createTargetPoint(
    idx, param.min_behavior_stop_margin, trajectory_vec, collision_point_vec, base_path);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (base_path.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point;
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint PointHelper::createTargetPoint(
  const int idx, const double margin, const Point2d & trajectory_vec,
  const Point2d & collision_point_vec,
  const Trajectory & base_path) const
{
  double length_sum = 0.0;
  length_sum += trajectory_vec.normalized().dot(collision_point_vec);
  Point2d line_start_point = autoware_utils::fromMsg(base_path.points.at(0).pose.position).to_2d();
  const double yaw = getYawFromQuaternion(base_path.points.at(0).pose.orientation);
  Point2d line_end_point {std::cos(yaw), std::sin(yaw)};

  StopPoint stop_point{0, Point2d {0.0, 0.0}};
  for (size_t j = idx; 0 < j; --j) {
    line_start_point = autoware_utils::fromMsg(base_path.points.at(j - 1).pose.position).to_2d();
    line_end_point = autoware_utils::fromMsg(base_path.points.at(j).pose.position).to_2d();
    if (margin < length_sum) {
      stop_point.index = j;
      break;
    }
    length_sum += (line_end_point - line_start_point).norm();
  }
  stop_point.point = getBackwardPointFromBasePoint(
    line_start_point, line_end_point, line_start_point, length_sum - margin);

  return stop_point;
}

SlowDownPoint PointHelper::createSlowDownStartPoint(
  const int idx, const double margin, const double slow_down_target_vel,
  const Point2d & trajectory_vec, const Point2d & slow_down_point_vec,
  const Trajectory & base_path,
  const double current_velocity_x,
  const SlowDownControlParameter & param) const
{
  double length_sum = trajectory_vec.normalized().dot(slow_down_point_vec);
  Point2d line_start_point = autoware_utils::fromMsg(base_path.points.at(0).pose.position).to_2d();
  const double yaw = getYawFromQuaternion(base_path.points.at(0).pose.orientation);
  Point2d line_end_point {std::cos(yaw), std::sin(yaw)};

  SlowDownPoint slow_down_point{0, Point2d {0.0, 0.0}, 0.0};
  for (size_t j = idx; 0 < j; --j) {
    line_start_point = autoware_utils::fromMsg(base_path.points.at(j).pose.position).to_2d();
    line_end_point = autoware_utils::fromMsg(base_path.points.at(j - 1).pose.position).to_2d();
    if (margin < length_sum) {
      slow_down_point.index = j;
      break;
    }
    length_sum += (line_end_point - line_start_point).norm();
  }
  slow_down_point.point = getBackwardPointFromBasePoint(
    line_start_point, line_end_point, line_start_point, length_sum - margin);

  slow_down_point.velocity = std::max(
    std::sqrt(
      slow_down_target_vel * slow_down_target_vel + 2 * param.max_deceleration *
      length_sum),
    current_velocity_x);
  return slow_down_point;
}

Trajectory PointHelper::insertSlowDownStartPoint(
  const SlowDownPoint & slow_down_start_point,
  const Trajectory & base_path,
  const Trajectory & input_path) const
{
  auto output_path = input_path;

  TrajectoryPoint slow_down_start_trajectory_point =
    base_path.points.at(std::max(static_cast<int>(slow_down_start_point.index) - 1, 0));
  slow_down_start_trajectory_point.pose.position =
    autoware_utils::toMsg(slow_down_start_point.point.to_3d());
  slow_down_start_trajectory_point.twist.linear.x = slow_down_start_point.velocity;
  output_path.points.insert(
    output_path.points.begin() + slow_down_start_point.index, slow_down_start_trajectory_point);
  return output_path;
}
}  // namespace obstacle_stop_planner
