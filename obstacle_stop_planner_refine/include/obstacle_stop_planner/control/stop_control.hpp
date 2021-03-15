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

#ifndef OBSTACLE_STOP_PLANNER__CONTROL__STOP_CONTROL_HPP_
#define OBSTACLE_STOP_PLANNER__CONTROL__STOP_CONTROL_HPP_

#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "boost/geometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
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

class StopController
{
public:
  explicit StopController(const StopControlParameter & param);

  Trajectory insertStopPoint(
    const size_t search_start_index,
    const Trajectory & base_path,
    const Point2d & nearest_collision_point,
    const Trajectory & input_msg);

  Polygon2d createVehiclePolygon(
    const Pose & current_pose, const Pose & next_pose, const VehicleInfo & vehicle_info);

  pcl::PointCloud<pcl::PointXYZ>::Ptr getCollisionPointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
    const Point2dPair & vehicle_center_points,
    const Polygon2d & one_step_polygon,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud);

  void clear();
  bool isCollision() const;

private:
  StopControlParameter param_;

  bool is_collision_ = false;
  size_t decimate_trajectory_collision_index_;
  Point3d nearest_collision_point_;
  rclcpp::Time nearest_collision_point_time_;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__CONTROL__STOP_CONTROL_HPP_
