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

#include "obstacle_stop_planner/trajectory.hpp"

namespace motion_planning
{
bool Trajectory::decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length, const VehicleInfo & vehicle_info,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  std::map<size_t /* decimate */, size_t /* origin */> index_map;
  return decimateTrajectory(input_trajectory, step_length, vehicle_info, output_trajectory, index_map);
}

bool Trajectory::decimateTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double step_length, const VehicleInfo & vehicle_info,
  autoware_planning_msgs::msg::Trajectory & output_trajectory,
  std::map<size_t /* decimate */, size_t /* origin */> & index_map)
{
  output_trajectory.header = input_trajectory.header;
  double trajectory_length_sum = 0.0;
  double next_length = 0.0;
  const double epsilon = 0.001;
  Eigen::Vector2d point1, point2;
  PointHelper point_helper;
  point_helper.SetVehicleInfo(vehicle_info);

  for (int i = 0; i < static_cast<int>(input_trajectory.points.size()) - 1; ++i) {
    if (next_length <= trajectory_length_sum + epsilon) {
      Eigen::Vector2d line_start_point, line_end_point, interpolated_point;
      line_start_point << input_trajectory.points.at(i).pose.position.x,
        input_trajectory.points.at(i).pose.position.y;
      line_end_point << input_trajectory.points.at(i + 1).pose.position.x,
        input_trajectory.points.at(i + 1).pose.position.y;
      point_helper.getBackwardPointFromBasePoint(
        line_start_point, line_end_point, line_end_point,
        -1.0 * (trajectory_length_sum - next_length), interpolated_point);
      autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
      trajectory_point = input_trajectory.points.at(i);
      trajectory_point.pose.position.x = interpolated_point.x();
      trajectory_point.pose.position.y = interpolated_point.y();
      output_trajectory.points.push_back(trajectory_point);
      index_map.insert(std::make_pair(output_trajectory.points.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }
    trajectory_length_sum += std::hypot(
      input_trajectory.points.at(i).pose.position.x - input_trajectory.points.at(i + 1).pose.position.x,
      input_trajectory.points.at(i).pose.position.y - input_trajectory.points.at(i + 1).pose.position.y);
  }
  if (!input_trajectory.points.empty()) {
    output_trajectory.points.push_back(input_trajectory.points.back());
    index_map.insert(
      std::make_pair(output_trajectory.points.size() - 1, input_trajectory.points.size() - 1));
  }
  return true;
}

bool Trajectory::trimTrajectoryWithIndexFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory, size_t & index)
{
  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input_trajectory.points.size(); ++i) {
    const double distance = std::hypot(
      input_trajectory.points.at(i).pose.position.x - self_pose.position.x,
      input_trajectory.points.at(i).pose.position.y - self_pose.position.y);
    if (!is_init || distance < min_distance) {
      is_init = true;
      min_distance = distance;
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input_trajectory.points.size(); ++i) {
    output_trajectory.points.push_back(input_trajectory.points.at(i));
  }
  output_trajectory.header = input_trajectory.header;
  index = min_distance_index;
  return true;
}

bool Trajectory::trimTrajectoryFromSelfPose(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const geometry_msgs::msg::Pose self_pose,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  size_t index;
  return trimTrajectoryWithIndexFromSelfPose(
    output_trajectory, self_pose, output_trajectory, index);
}

bool Trajectory::extendTrajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory,
  const double extend_distance,
  autoware_planning_msgs::msg::Trajectory & output_trajectory)
{
  output_trajectory = input_trajectory;
  const auto goal_point = input_trajectory.points.back();
  double interpolation_distance = 0.1;
  PointHelper point_helper;

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = point_helper.getExtendTrajectoryPoint(extend_sum, goal_point);
    output_trajectory.points.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = point_helper.getExtendTrajectoryPoint(extend_distance, goal_point);
  output_trajectory.points.push_back(extend_trajectory_point);

  return true;
}

}  // namespace motion_planning
