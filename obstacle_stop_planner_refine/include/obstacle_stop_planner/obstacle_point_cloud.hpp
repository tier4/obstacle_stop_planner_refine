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
#include "vehicle_info_util/vehicle_info_util.hpp"

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
    no_height_pointcloud_ptr->emplace_back(point.x, point.y, 0.0);
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05F, 0.05F, 100000.0F);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr);
  obstacle_ros_pointcloud_ptr->header = msg->header;
  return obstacle_ros_pointcloud_ptr;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr transformObstacle(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & obstacle_ros_pointcloud_ptr,
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
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

  return transformed_obstacle_pointcloud_ptr;
}

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_POINT_CLOUD_HPP_
