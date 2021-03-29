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

#ifndef OBSTACLE_STOP_PLANNER__OBSTACLE_POINT_CLOUD_HPP_
#define OBSTACLE_STOP_PLANNER__OBSTACLE_POINT_CLOUD_HPP_

#include <memory>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

namespace obstacle_stop_planner
{
inline sensor_msgs::msg::PointCloud2::ConstSharedPtr updatePointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  auto obstacle_ros_pointcloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_filtered_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*msg, *pointcloud_ptr);

  for (const auto & point : pointcloud_ptr->points) {
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05F, 0.05F, 100000.0F);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr);
  obstacle_ros_pointcloud_ptr->header = msg->header;
  return obstacle_ros_pointcloud_ptr;
}

inline static pcl::PointCloud<pcl::PointXYZ>::Ptr searchPointcloudNearTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
  const double search_radius,
  const VehicleInfo & param)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  const double squared_radius = search_radius * search_radius;
  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = getVehicleCenterFromBase(
      trajectory_point.pose,
      param.vehicle_length,
      param.rear_overhang);

    for (const auto & point : input_pointcloud_ptr->points) {
      const double x = center_pose.position.x - point.x;
      const double y = center_pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {output_pointcloud_ptr->points.push_back(point);}
    }
  }
  return output_pointcloud_ptr;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr searchCandidateObstacle(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & obstacle_ros_pointcloud_ptr,
  const tf2_ros::Buffer & tf_buffer,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const double search_radius,
  const VehicleInfo & param,
  const rclcpp::Logger & logger)
{
  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
      trajectory.header.frame_id, obstacle_ros_pointcloud_ptr->header.frame_id,
      obstacle_ros_pointcloud_ptr->header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      logger,
      "[obstacle_stop_planner] Failed to look up transform from " <<
        trajectory.header.frame_id << " to " << obstacle_ros_pointcloud_ptr->header.frame_id);
    // do not publish path
    return nullptr;
  }

  Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pointcloud_pcl_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr, *obstacle_pointcloud_pcl_ptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_obstacle_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *obstacle_pointcloud_pcl_ptr,
    *transformed_obstacle_pointcloud_ptr,
    affine_matrix);

  // search obstacle candidate pointcloud to reduce calculation cost
  auto obstacle_candidate_pointcloud_ptr = searchPointcloudNearTrajectory(
    trajectory, transformed_obstacle_pointcloud_ptr,
    search_radius,
    param);
  obstacle_candidate_pointcloud_ptr->header = transformed_obstacle_pointcloud_ptr->header;
  return obstacle_candidate_pointcloud_ptr;
}

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_POINT_CLOUD_HPP_
