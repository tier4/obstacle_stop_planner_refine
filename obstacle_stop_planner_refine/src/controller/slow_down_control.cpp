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

#include <algorithm>

#include "obstacle_stop_planner/control/slow_down_control.hpp"
#include "obstacle_stop_planner/util/one_step_polygon.hpp"
#include "obstacle_stop_planner/util/point_helper.hpp"

namespace obstacle_stop_planner
{
SlowDownController::SlowDownController(const SlowDownControlParameter & param)
: param_(param)
{
}

Polygon2d SlowDownController::createVehiclePolygon(
  const Pose & current_pose,
  const Pose & next_pose,
  const VehicleInfo & vehicle_info)
{
  if (param_.enable_slow_down) {
    return createOneStepPolygon(
      current_pose,
      next_pose,
      param_.expand_slow_down_range,
      vehicle_info);
  } else {
    return Polygon2d();
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SlowDownController::getSlowDownPointcloud(
  const bool is_slow_down,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud,
  const Point2dPair & vehicle_center_points,
  const Polygon2d & one_step_polygon)
{
  if (!is_slow_down && param_.enable_slow_down) {
    for (size_t j = 0; j < obstacle_candidate_pointcloud->size(); ++j) {
      const auto point_xyz = obstacle_candidate_pointcloud->at(j);
      Point2d point2 {point_xyz.x, point_xyz.y};
      if (
        distance(vehicle_center_points.prev, point2) < param_.slow_down_search_radius ||
        distance(vehicle_center_points.next, point2) < param_.slow_down_search_radius)
      {
        if (within(point2, one_step_polygon)) {
          slow_down_pointcloud_ptr_->push_back(point_xyz);
          candidate_slow_down_ = true;
        }
      }
    }
  } else {
    slow_down_pointcloud_ptr_ = obstacle_candidate_pointcloud;
  }
  return slow_down_pointcloud_ptr_;
}

void SlowDownController::clear()
{
  candidate_slow_down_ = false;
}

bool SlowDownController::candidateSlowDown() const
{
  return candidate_slow_down_;
}

Trajectory SlowDownController::insertSlowDownPoint(
  const size_t search_start_index,
  const Trajectory & base_path,
  const Point2d & nearest_slow_down_point,
  const double lateral_deviation,
  const double current_velocity_x,
  const double vehicle_width,
  const Trajectory & input_msg)
{
  auto output_msg = input_msg;
  PointHelper point_helper;
  const auto slow_down_target_vel = calcSlowDownTargetVel(lateral_deviation, vehicle_width);

  for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
    const auto base_pose = base_path.points.at(i).pose;
    const double yaw =
      getYawFromQuaternion(base_pose.orientation);
    const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
    const Point2d slow_down_point_vec {
      nearest_slow_down_point.x() - base_pose.position.x,
      nearest_slow_down_point.y() - base_pose.position.y};

    if (
      trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
      (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec)))
    {
      const auto slow_down_start_point = point_helper.createSlowDownStartPoint(
        i, param_.slow_down_margin, slow_down_target_vel, trajectory_vec, slow_down_point_vec,
        base_path, current_velocity_x, param_);

      if (slow_down_start_point.index <= output_msg.points.size()) {
        // TrajectoryPoint slowdown_trajectory_point;
        // std::tie(slowdown_trajectory_point, output_msg) =
        output_msg = point_helper.insertSlowDownStartPoint(
          slow_down_start_point, base_path, output_msg);
        // debug_ptr_->pushPose(slowdown_trajectory_point.pose, PoseType::SlowDownStart);
        output_msg = insertSlowDownVelocity(
          slow_down_start_point.index, slow_down_target_vel, slow_down_start_point.velocity,
          output_msg);
      }
      break;
    }
  }
  return output_msg;
}

Trajectory SlowDownController::insertSlowDownVelocity(
  const size_t slow_down_start_point_idx,
  const double slow_down_target_vel,
  const double initial_slow_down_vel,
  const Trajectory & input_path)
{
  // TrajectoryPoint slow_down_end_trajectory_point;
  auto output_path = input_path;
  bool is_slow_down_end = false;
  double slow_down_vel = initial_slow_down_vel;

  for (size_t j = slow_down_start_point_idx; j < output_path.points.size() - 1; ++j) {
    output_path.points.at(j).twist.linear.x =
      std::min(slow_down_vel, output_path.points.at(j).twist.linear.x);
    const auto dist = std::hypot(
      output_path.points.at(j).pose.position.x - output_path.points.at(j + 1).pose.position.x,
      output_path.points.at(j).pose.position.y - output_path.points.at(j + 1).pose.position.y);
    slow_down_vel = std::max(
      slow_down_target_vel,
      std::sqrt(
        std::max(
          slow_down_vel * slow_down_vel - 2 * param_.max_deceleration * dist,
          0.0)));
    if (!is_slow_down_end && slow_down_vel <= slow_down_target_vel) {
      // slow_down_end_trajectory_point = output_path.points.at(j + 1);
      is_slow_down_end = true;
    }
  }
  // if (!is_slow_down_end) {
  //   slow_down_end_trajectory_point = output_path.points.back();
  // }
  // debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
  return output_path;
}

double SlowDownController::calcSlowDownTargetVel(
  const double lateral_deviation,
  const double vehicle_width)
{
  return param_.min_slow_down_vel +
         (param_.max_slow_down_vel - param_.min_slow_down_vel) *
         std::max(lateral_deviation - vehicle_width / 2, 0.0) /
         param_.expand_slow_down_range;
}

}  // namespace obstacle_stop_planner
