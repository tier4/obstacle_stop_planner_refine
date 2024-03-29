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
#include <tuple>
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace obstacle_stop_planner
{

struct DecimateTrajectoryMap
{
  autoware_planning_msgs::msg::Trajectory orig_trajectory;
  autoware_planning_msgs::msg::Trajectory decimate_trajectory;
  std::map<size_t /* decimate */, size_t /* origin */> index_map;
};

autoware_planning_msgs::msg::Trajectory extendTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const double extend_distance);
DecimateTrajectoryMap decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length);
std::tuple<autoware_planning_msgs::msg::Trajectory, size_t> trimTrajectoryWithIndexFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose & self_pose);
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__TRAJECTORY_HPP_
