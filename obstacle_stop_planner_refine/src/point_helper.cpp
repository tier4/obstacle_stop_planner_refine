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

#include "obstacle_stop_planner/point_helper.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace obstacle_stop_planner
{
bool PointHelper::getBackwardPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length,
  Eigen::Vector2d & output_point) const
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

void PointHelper::getNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time) const
{
  double min_norm = 0.0;
  bool is_init = false;
  const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
  Eigen::Vector2d base_pose_vec;
  base_pose_vec << std::cos(yaw), std::sin(yaw);

  for (size_t i = 0; i < pointcloud.size(); ++i) {
    Eigen::Vector2d pointcloud_vec;
    pointcloud_vec << pointcloud.at(i).x - base_pose.position.x,
      pointcloud.at(i).y - base_pose.position.y;
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = pointcloud.at(i);
      *nearest_collision_point_time = pcl_conversions::fromPCL(pointcloud.header).stamp;
      is_init = true;
    }
  }
}

void PointHelper::getLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * lateral_nearest_point, double * deviation) const
{
  double min_norm = std::numeric_limits<double>::max();
  const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
  Eigen::Vector2d base_pose_vec;
  base_pose_vec << std::cos(yaw), std::sin(yaw);
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    Eigen::Vector2d pointcloud_vec;
    pointcloud_vec << pointcloud.at(i).x - base_pose.position.x,
      pointcloud.at(i).y - base_pose.position.y;
    double norm =
      std::abs(base_pose_vec.x() * pointcloud_vec.y() - base_pose_vec.y() * pointcloud_vec.x());
    if (norm < min_norm) {
      min_norm = norm;
      *lateral_nearest_point = pointcloud.at(i);
    }
  }
  *deviation = min_norm;
}

autoware_planning_msgs::msg::TrajectoryPoint PointHelper::insertStopPoint(
  const StopPoint & stop_point, const autoware_planning_msgs::msg::Trajectory & base_path,
  autoware_planning_msgs::msg::Trajectory & output_path) const
{
  autoware_planning_msgs::msg::TrajectoryPoint stop_trajectory_point =
    base_path.points.at(std::max(static_cast<int>(stop_point.index) - 1, 0));
  stop_trajectory_point.pose.position.x = stop_point.point.x();
  stop_trajectory_point.pose.position.y = stop_point.point.y();
  stop_trajectory_point.twist.linear.x = 0.0;
  output_path.points.insert(output_path.points.begin() + stop_point.index, stop_trajectory_point);
  for (size_t j = stop_point.index; j < output_path.points.size(); ++j) {
    output_path.points.at(j).twist.linear.x = 0.0;
  }
  return stop_trajectory_point;
}

StopPoint PointHelper::searchInsertPoint(
  const int idx, const autoware_planning_msgs::msg::Trajectory & base_path,
  const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & collision_point_vec) const
{
  const auto max_dist_stop_point =
    createTargetPoint(
    idx, vehicle_info_->stop_margin_, trajectory_vec, collision_point_vec,
    base_path);
  const auto min_dist_stop_point = createTargetPoint(
    idx, vehicle_info_->min_behavior_stop_margin_, trajectory_vec, collision_point_vec, base_path);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (base_path.points.at(std::max(j, 0)).twist.linear.x == 0.0) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point;
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint PointHelper::createTargetPoint(
  const int idx, const double margin, const Eigen::Vector2d & trajectory_vec,
  const Eigen::Vector2d & collision_point_vec,
  const autoware_planning_msgs::msg::Trajectory & base_path) const
{
  double length_sum = 0.0;
  length_sum += trajectory_vec.normalized().dot(collision_point_vec);
  Eigen::Vector2d line_start_point, line_end_point;
  {
    line_start_point << base_path.points.at(0).pose.position.x,
      base_path.points.at(0).pose.position.y;
    const double yaw = getYawFromGeometryMsgsQuaternion(base_path.points.at(0).pose.orientation);
    line_end_point << std::cos(yaw), std::sin(yaw);
  }

  StopPoint stop_point{0, Eigen::Vector2d()};
  for (size_t j = idx; 0 < j; --j) {
    line_start_point << base_path.points.at(j - 1).pose.position.x,
      base_path.points.at(j - 1).pose.position.y;
    line_end_point << base_path.points.at(j).pose.position.x,
      base_path.points.at(j).pose.position.y;
    if (margin < length_sum) {
      stop_point.index = j;
      break;
    }
    length_sum += (line_end_point - line_start_point).norm();
  }
  getBackwardPointFromBasePoint(
    line_start_point, line_end_point, line_start_point, length_sum - margin, stop_point.point);

  return stop_point;
}

SlowDownPoint PointHelper::createSlowDownStartPoint(
  const int idx, const double margin, const double slow_down_target_vel,
  const Eigen::Vector2d & trajectory_vec, const Eigen::Vector2d & slow_down_point_vec,
  const autoware_planning_msgs::msg::Trajectory & base_path,
  const double current_velocity_x) const
{
  double length_sum = 0.0;
  length_sum += trajectory_vec.normalized().dot(slow_down_point_vec);
  Eigen::Vector2d line_start_point{};
  Eigen::Vector2d line_end_point{};

  SlowDownPoint slow_down_point{0, Eigen::Vector2d(), 0.0};
  for (size_t j = idx; 0 < j; --j) {
    line_start_point << base_path.points.at(j).pose.position.x,
      base_path.points.at(j).pose.position.y;
    line_end_point << base_path.points.at(j - 1).pose.position.x,
      base_path.points.at(j - 1).pose.position.y;
    if (margin < length_sum) {
      slow_down_point.index = j;
      break;
    }
    length_sum += (line_end_point - line_start_point).norm();
  }
  const double backward_length = length_sum - margin;
  if (backward_length < 0) {
    slow_down_point.index = 0;
    slow_down_point.point = Eigen::Vector2d(
      base_path.points.at(0).pose.position.x, base_path.points.at(0).pose.position.y);
  } else {
    getBackwardPointFromBasePoint(
      line_start_point, line_end_point, line_start_point, backward_length, slow_down_point.point);
  }

  slow_down_point.velocity = std::max(
    std::sqrt(
      slow_down_target_vel * slow_down_target_vel + 2 * vehicle_info_->max_deceleration_ *
      length_sum),
    current_velocity_x);
  return slow_down_point;
}

autoware_planning_msgs::msg::TrajectoryPoint PointHelper::insertSlowDownStartPoint(
  const SlowDownPoint & slow_down_start_point,
  const autoware_planning_msgs::msg::Trajectory & base_path,
  autoware_planning_msgs::msg::Trajectory & output_path) const
{
  autoware_planning_msgs::msg::TrajectoryPoint slow_down_start_trajectory_point =
    base_path.points.at(std::max(static_cast<int>(slow_down_start_point.index) - 1, 0));
  slow_down_start_trajectory_point.pose.position.x = slow_down_start_point.point.x();
  slow_down_start_trajectory_point.pose.position.y = slow_down_start_point.point.y();
  slow_down_start_trajectory_point.twist.linear.x = slow_down_start_point.velocity;
  constexpr double epsilon = 0.001;
  const auto & insert_target_point = output_path.points.at(slow_down_start_point.index);
  if (
    autoware_utils::calcDistance2d(slow_down_start_trajectory_point, insert_target_point) >
    epsilon)
  {
    output_path.points.insert(
      output_path.points.begin() + slow_down_start_point.index, slow_down_start_trajectory_point);
  }
  return slow_down_start_trajectory_point;
}

autoware_planning_msgs::msg::TrajectoryPoint PointHelper::getExtendTrajectoryPoint(
  const double extend_distance,
  const autoware_planning_msgs::msg::TrajectoryPoint & goal_point) const
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  geometry_msgs::msg::Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  autoware_planning_msgs::msg::TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.twist = goal_point.twist;
  extend_trajectory_point.accel = goal_point.accel;
  return extend_trajectory_point;
}

}  // namespace obstacle_stop_planner
