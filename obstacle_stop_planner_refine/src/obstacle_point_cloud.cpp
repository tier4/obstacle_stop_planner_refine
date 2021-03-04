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

#include "obstacle_stop_planner/obstacle_point_cloud.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"

namespace obstacle_stop_planner
{
ObstaclePointCloud::ObstaclePointCloud(rclcpp::Logger logger)
: logger_(logger)
{
}

void ObstaclePointCloud::setPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  obstacle_ros_pointcloud_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
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
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = msg->header;
}

void ObstaclePointCloud::setSearchRadius(const double value)
{
  search_radius_ = value;
}

void ObstaclePointCloud::setVehicleInfo(const VehicleInfo vehicle_info)
{
  vehicle_info_ = std::make_shared<VehicleInfo>(vehicle_info);
}

bool ObstaclePointCloud::isDataReceived()
{
  return obstacle_ros_pointcloud_ptr_ != nullptr ? true : false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObstaclePointCloud::searchCandidateObstacle(const tf2_ros::Buffer & tf_buffer, const autoware_planning_msgs::msg::Trajectory & trajectory)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
      trajectory.header.frame_id, obstacle_ros_pointcloud_ptr_->header.frame_id,
      obstacle_ros_pointcloud_ptr_->header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "[obstacle_stop_planner] Failed to look up transform from " <<
        trajectory.header.frame_id << " to " << obstacle_ros_pointcloud_ptr_->header.frame_id);
    // do not publish path
    return nullptr;
  }

  Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pointcloud_pcl_ptr_(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*obstacle_ros_pointcloud_ptr_, *obstacle_pointcloud_pcl_ptr_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_obstacle_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(
    *obstacle_pointcloud_pcl_ptr_, *transformed_obstacle_pointcloud_ptr,
    affine_matrix);

  // search obstacle candidate pointcloud to reduce calculation cost
  searchPointcloudNearTrajectory(
    trajectory, transformed_obstacle_pointcloud_ptr,
    obstacle_candidate_pointcloud_ptr);
  obstacle_candidate_pointcloud_ptr->header = transformed_obstacle_pointcloud_ptr->header;
  return obstacle_candidate_pointcloud_ptr;
}

bool ObstaclePointCloud::searchPointcloudNearTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr)
{
  const double squared_radius = search_radius_ * search_radius_;
  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = vehicle_info_->getVehicleCenterFromBase(trajectory_point.pose);
    for (const auto & point : input_pointcloud_ptr->points) {
      const double x = center_pose.position.x - point.x;
      const double y = center_pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {output_pointcloud_ptr->points.push_back(point);}
    }
  }
  return true;
}

}  // namespace obstacle_stop_planner
