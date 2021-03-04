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

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "geometry_msgs/msg/pose.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "obstacle_stop_planner/util.hpp"
#include "obstacle_stop_planner/vehicle.hpp"

#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"


namespace obstacle_stop_planner {

struct StopPoint
{
  size_t index;
  Eigen::Vector2d point;
};

struct SlowDownPoint
{
  size_t index;
  Eigen::Vector2d point;
  double velocity;
};

class PointHelper
{
public:
  void SetVehicleInfo(const VehicleInfo vehicle_info) { vehicle_info_ = std::make_shared<VehicleInfo>(vehicle_info); }
  void SetVehicleInfo(std::shared_ptr<VehicleInfo> vehicle_info) { vehicle_info_ = vehicle_info; }

  bool getBackwardPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point) const;
  void getNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time) const;
  void getLateralNearestPoint(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
    pcl::PointXYZ * lateral_nearest_point, double * deviation) const;

  autoware_planning_msgs::msg::TrajectoryPoint insertStopPoint(
    const StopPoint & stop_point, const autoware_planning_msgs::msg::Trajectory & base_path,
    autoware_planning_msgs::msg::Trajectory & output_path) const;

  StopPoint searchInsertPoint(
    const int idx, const autoware_planning_msgs::msg::Trajectory & base_path,
    const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & collision_point_vec) const;

  StopPoint createTargetPoint(
    const int idx, const double margin, const Eigen::Vector2d & trajectory_vec,
    const Eigen::Vector2d & collision_point_vec,
    const autoware_planning_msgs::msg::Trajectory & base_path) const;

  SlowDownPoint createSlowDownStartPoint(
    const int idx, const double margin, const double slow_down_target_vel,
    const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & slow_down_point_vec,
    const autoware_planning_msgs::msg::Trajectory & base_path,
    const double current_velocity_x) const;

  autoware_planning_msgs::msg::TrajectoryPoint insertSlowDownStartPoint(
    const SlowDownPoint & slow_down_start_point,
    const autoware_planning_msgs::msg::Trajectory & base_path,
    autoware_planning_msgs::msg::Trajectory & output_path) const;

  autoware_planning_msgs::msg::TrajectoryPoint getExtendTrajectoryPoint(
    double extend_distance, const autoware_planning_msgs::msg::TrajectoryPoint & goal_point) const;

private:
  std::shared_ptr<VehicleInfo> vehicle_info_;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__POINT_HELPER_HPP_
