// Copyright 2019 Autoware Foundation
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
#ifndef OBSTACLE_STOP_PLANNER__NODE_HPP_
#define OBSTACLE_STOP_PLANNER__NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "obstacle_stop_planner/visibility_control.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/expand_stop_range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "obstacle_stop_planner/obstacle_stop_planner.hpp"

namespace obstacle_stop_planner
{
class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & options);

private:
  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    dynamic_object_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::ExpandStopRange>::SharedPtr
    expand_stop_range_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;

  /*
   * Parameter
   */
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr_;
  rclcpp::Time prev_col_point_time_;
  pcl::PointXYZ prev_col_point_;
  bool pointcloud_received_;
  bool current_velocity_reveived_;

  std::unique_ptr<obstacle_stop_planner::ObstacleStopPlanner> planner_;

  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void pathCallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);
  void externalExpandStopRangeCallback(
    const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg);
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
