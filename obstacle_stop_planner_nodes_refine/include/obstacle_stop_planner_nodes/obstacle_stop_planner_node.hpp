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

#ifndef OBSTACLE_STOP_PLANNER_NODES__OBSTACLE_STOP_PLANNER_NODE_HPP_
#define OBSTACLE_STOP_PLANNER_NODES__OBSTACLE_STOP_PLANNER_NODE_HPP_

#include <obstacle_stop_planner_nodes/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "obstacle_stop_planner/obstacle_stop_planner.hpp"

namespace obstacle_stop_planner_nodes
{

using rclcpp::Subscription;
using rclcpp::Publisher;
using autoware_planning_msgs::msg::Trajectory;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::Pose;

class OBSTACLE_STOP_PLANNER_NODES_PUBLIC ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & options);

private:
  void obstaclePointcloudCallback(const PointCloud2::SharedPtr input_msg);
  void pathCallback(const Trajectory::SharedPtr input_msg);
  void externalExpandStopRangeCallback(const float expand_stop_range);
  Pose getSelfPose(const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer);
  geometry_msgs::msg::TransformStamped getTransform(
    const std_msgs::msg::Header & target,
    const std_msgs::msg::Header & source,
    const tf2_ros::Buffer & tf_buffer);

  std::unique_ptr<obstacle_stop_planner::ObstacleStopPlanner> planner_;
    // publisher and subscriber
  Subscription<Trajectory>::SharedPtr path_sub_;
  Subscription<PointCloud2>::SharedPtr obstacle_pointcloud_sub_;
  Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_velocity_sub_;
  Subscription<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr
    dynamic_object_sub_;
  Subscription<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr expand_stop_range_sub_;
  Publisher<Trajectory>::SharedPtr path_pub_;

  geometry_msgs::msg::TwistStamped::SharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::SharedPtr object_ptr_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool is_pointcloud_received_;
  std_msgs::msg::Header pointcloud_header_;
  bool enable_slow_down_;
};

}  // namespace obstacle_stop_planner_nodes

#endif  // OBSTACLE_STOP_PLANNER_NODES__OBSTACLE_STOP_PLANNER_NODE_HPP_
