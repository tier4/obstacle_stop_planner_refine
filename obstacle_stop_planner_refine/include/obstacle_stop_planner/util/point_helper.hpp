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

#ifndef OBSTACLE_STOP_PLANNER__POINT_HELPER_HPP_
#define OBSTACLE_STOP_PLANNER__POINT_HELPER_HPP_

#include <memory>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "obstacle_stop_planner/util/util.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"


namespace obstacle_stop_planner
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

struct StopPoint
{
  size_t index;
  Point2d point;
};

struct SlowDownPoint
{
  size_t index;
  Point2d point;
  double velocity;
};

struct PointStamped
{
  rclcpp::Time time;
  Point2d point;
};

struct PointDeviation
{
  double deviation;
  pcl::PointXYZ point;
};

class PointHelper
{
public:
  Point2d getBackwardPointFromBasePoint(
    const Point2d & line_point1, const Point2d & line_point2,
    const Point2d & base_point, const double backward_length) const;
  PointStamped getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
    const Pose & base_pose) const;
  PointDeviation getLateralNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
    const Pose & base_pose) const;

  Trajectory insertStopPoint(
    const StopPoint & stop_point, const Trajectory & base_path,
    const Trajectory & input_path) const;

  StopPoint searchInsertPoint(
    const int idx, const Trajectory & base_path,
    const Point2d & trajectory_vec, const Point2d & collision_point_vec, const StopControlParameter & param) const;

  StopPoint createTargetPoint(
    const int idx, const double margin, const Point2d & trajectory_vec,
    const Point2d & collision_point_vec,
    const Trajectory & base_path) const;

  SlowDownPoint createSlowDownStartPoint(
    const int idx, const double margin, const double slow_down_target_vel,
    const Point2d & trajectory_vec, const Point2d & slow_down_point_vec,
    const Trajectory & base_path,
    const double current_velocity_x, const SlowDownControlParameter & param) const;

  Trajectory insertSlowDownStartPoint(
    const SlowDownPoint & slow_down_start_point,
    const Trajectory & base_path,
    const Trajectory & input_path) const;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__POINT_HELPER_HPP_
