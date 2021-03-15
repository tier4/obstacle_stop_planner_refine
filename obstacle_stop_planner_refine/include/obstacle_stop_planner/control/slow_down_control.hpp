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

#ifndef OBSTACLE_STOP_PLANNER__CONTROL__SLOW_DOWN_CONTROLLER_HPP_
#define OBSTACLE_STOP_PLANNER__CONTROL__SLOW_DOWN_CONTROLLER_HPP_

#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "obstacle_stop_planner/util/util.hpp"

namespace obstacle_stop_planner
{
using geometry_msgs::msg::Pose;
using autoware_utils::Point2d;
using autoware_utils::Point3d;
using autoware_utils::Polygon2d;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using boost::geometry::distance;
using boost::geometry::within;

class SlowDownController
{
public:
  explicit SlowDownController(const SlowDownControlParameter & param);

  Trajectory insertSlowDownPoint(
    const size_t search_start_index,
    const Trajectory & base_path,
    const Point2d & nearest_slow_down_point,
    const double lateral_deviation,
    const double current_velocity_x,
    const double vehicle_width,
    const Trajectory & input_msg);

  Polygon2d createVehiclePolygon(
    const Pose & current_pose, const Pose & next_pose, const VehicleInfo & vehicle_info);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getSlowDownPointcloud(
    const bool is_slow_down,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud,
    const Point2dPair & vehicle_center_points,
    const Polygon2d & one_step_polygon);

  void clear();
  bool candidateSlowDown() const;

private:
  Trajectory insertSlowDownVelocity(
    const size_t slow_down_start_point_idx,
    const double slow_down_target_vel,
    const double initial_slow_down_vel,
    const Trajectory & input_path);

  double calcSlowDownTargetVel(const double lateral_deviation, const double vehicle_width);

  SlowDownControlParameter param_;

  bool candidate_slow_down_ = false;
  size_t decimate_trajectory_slow_down_index_;
  Point3d nearest_slow_down_point_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr_;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__CONTROL__SLOW_DOWN_CONTROLLER_HPP_
