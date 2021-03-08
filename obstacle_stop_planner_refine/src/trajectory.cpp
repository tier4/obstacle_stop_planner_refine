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

#include <map>
#include <utility>
#include <tuple>
#include "boost/geometry.hpp"
#include "obstacle_stop_planner/trajectory.hpp"
#include "obstacle_stop_planner/point_helper.hpp"
#include "obstacle_stop_planner/util.hpp"

namespace obstacle_stop_planner
{
DecimateTrajectoryMap decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length,
  const Param & param)
{
  DecimateTrajectoryMap output;
  output.orig_trajectory = input_trajectory;
  output.decimate_trajectory.header = input_trajectory.header;
  double trajectory_length_sum = 0.0;
  double next_length = 0.0;
  const double epsilon = 0.001;
  PointHelper point_helper {param};

  for (int i = 0; i < static_cast<int>(input_trajectory.points.size()) - 1; ++i) {
    if (next_length <= trajectory_length_sum + epsilon) {
      const auto line_start_point = autoware_utils::fromMsg(
        input_trajectory.points.at(i).pose.position).to_2d();
      const auto line_end_point = autoware_utils::fromMsg(
        input_trajectory.points.at(i + 1).pose.position).to_2d();
      Point2d interpolated_point;
      point_helper.getBackwardPointFromBasePoint(
        line_start_point, line_end_point, line_end_point,
        -1.0 * (trajectory_length_sum - next_length), interpolated_point);
      autoware_planning_msgs::msg::TrajectoryPoint trajectory_point = input_trajectory.points.at(i);
      trajectory_point.pose.position = autoware_utils::toMsg(
        interpolated_point.to_3d(input_trajectory.points.at(i).pose.position.z));
      output.decimate_trajectory.points.emplace_back(trajectory_point);
      output.index_map.insert(
        std::make_pair(output.decimate_trajectory.points.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }

    const auto p1 = autoware_utils::fromMsg(input_trajectory.points.at(i).pose.position);
    const auto p2 = autoware_utils::fromMsg(input_trajectory.points.at(i + 1).pose.position);
    trajectory_length_sum += boost::geometry::distance(p1.to_2d(), p2.to_2d());
  }
  if (!input_trajectory.points.empty()) {
    output.decimate_trajectory.points.emplace_back(input_trajectory.points.back());
    output.index_map.insert(
      std::make_pair(
        output.decimate_trajectory.points.size() - 1,
        input_trajectory.points.size() - 1));
  }
  return output;
}

std::tuple<autoware_planning_msgs::msg::Trajectory, size_t>
trimTrajectoryWithIndexFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose & self_pose)
{
  autoware_planning_msgs::msg::Trajectory output_trajectory;
  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input_trajectory.points.size(); ++i) {
    const auto p1 = autoware_utils::fromMsg(input_trajectory.points.at(i).pose.position);
    const auto p2 = autoware_utils::fromMsg(self_pose.position);
    const double point_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    if (!is_init || point_distance < min_distance) {
      is_init = true;
      min_distance = point_distance;
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input_trajectory.points.size(); ++i) {
    output_trajectory.points.emplace_back(input_trajectory.points.at(i));
  }
  output_trajectory.header = input_trajectory.header;
  return std::forward_as_tuple(output_trajectory, min_distance_index);
}
}  // namespace obstacle_stop_planner
