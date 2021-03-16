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

#include "obstacle_stop_planner/control/stop_control.hpp"
#include "obstacle_stop_planner/util/one_step_polygon.hpp"
#include "obstacle_stop_planner/util/point_helper.hpp"

namespace obstacle_stop_planner
{
StopController::StopController(const StopControlParameter & param)
: param_(param)
{
}

Polygon2d StopController::createVehiclePolygon(
  const Pose & current_pose, const Pose & next_pose, const VehicleInfo & vehicle_info)
{
  return createOneStepPolygon(
    current_pose,
    next_pose,
    param_.expand_stop_range,
    vehicle_info);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StopController::getCollisionPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
  const Point2dPair & vehicle_center_points,
  const Polygon2d & one_step_polygon,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud)
{
  auto output_pointcloud = input_pointcloud;
  for (size_t j = 0; j < slow_down_pointcloud->size(); ++j) {
    Point2d point(slow_down_pointcloud->at(j).x, slow_down_pointcloud->at(j).y);
    if (
      distance(vehicle_center_points.prev, point) < param_.stop_search_radius ||
      distance(vehicle_center_points.next, point) < param_.stop_search_radius)
    {
      if (within(point, one_step_polygon)) {
        output_pointcloud->push_back(slow_down_pointcloud->at(j));
        is_collision_ = true;
        // debug_ptr_->pushPolygon(
        //   one_step_polygon, trajectory_point.pose.position.z,
        //   PolygonType::Collision);
      }
    }
  }
  return output_pointcloud;
}

void StopController::updateParameter(const StopControlParameter & param)
{
  param_ = param;
}

void StopController::clear()
{
  is_collision_ = false;
}

bool StopController::isCollision() const
{
  return is_collision_;
}

Trajectory StopController::insertStopPoint(
  const size_t search_start_index,
  const Trajectory & base_path,
  const Point2d & nearest_collision_point,
  const Trajectory & input_msg)
{
  auto output_msg = input_msg;
  PointHelper point_helper;

  for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
    const auto base_pose = base_path.points.at(i).pose;
    const double yaw =
      getYawFromQuaternion(base_pose.orientation);
    const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
    const Point2d collision_point_vec {
      nearest_collision_point.x() - base_pose.position.x,
      nearest_collision_point.y() - base_pose.position.y};

    if (
      trajectory_vec.dot(collision_point_vec) < 0.0 ||
      (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec)))
    {
      const auto stop_point =
        point_helper.searchInsertPoint(i, base_path, trajectory_vec, collision_point_vec, param_);
      if (stop_point.index <= output_msg.points.size()) {
        // TrajectoryPoint trajectory_point;
        // std::tie(trajectory_point, output_msg) =
        output_msg = point_helper.insertStopPoint(stop_point, base_path, output_msg);
        // debug_ptr_->pushPose(trajectory_point.pose, PoseType::Stop);
      }
      break;
    }
  }
  return output_msg;
}


}  // namespace obstacle_stop_planner
