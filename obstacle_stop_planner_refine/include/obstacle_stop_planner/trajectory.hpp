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

#ifndef OBSTACLE_STOP_PLANNER__TRAJECTORY_HPP_
#define OBSTACLE_STOP_PLANNER__TRAJECTORY_HPP_

#include <map>
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "obstacle_stop_planner/vehicle.hpp"
#include "obstacle_stop_planner/point_helper.hpp"

namespace obstacle_stop_planner {

class Trajectory
{
public:
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length, const VehicleInfo & vehicle_info,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool decimateTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length, const VehicleInfo & vehicle_info,
    autoware_planning_msgs::msg::Trajectory & output_trajectory,
    std::map<size_t /* decimate */, size_t /* origin */> & index_map);
  bool trimTrajectoryFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
  bool trimTrajectoryWithIndexFromSelfPose(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const geometry_msgs::msg::Pose self_pose,
    autoware_planning_msgs::msg::Trajectory & output_trajectory,
    size_t & index);
  bool extendTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const double extend_distance,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__TRAJECTORY_HPP_
