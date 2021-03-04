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

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "tf2_ros/transform_listener.h"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "obstacle_stop_planner/vehicle.hpp"

namespace obstacle_stop_planner
{

class ObstaclePointCloud
{
public:
  explicit ObstaclePointCloud(rclcpp::Logger logger);

  void setPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void setSearchRadius(const double value);
  void setVehicleInfo(const VehicleInfo vehicle_info);

  bool isDataReceived();
  pcl::PointCloud<pcl::PointXYZ>::Ptr searchCandidateObstacle(
    const tf2_ros::Buffer & tf_buffer,
    const autoware_planning_msgs::msg::Trajectory & trajectory);

private:
  bool searchPointcloudNearTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pointcloud_ptr);

  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_;
  rclcpp::Logger logger_;
  double search_radius_;
  std::shared_ptr<VehicleInfo> vehicle_info_;
};

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_POINT_CLOUD_HPP_
