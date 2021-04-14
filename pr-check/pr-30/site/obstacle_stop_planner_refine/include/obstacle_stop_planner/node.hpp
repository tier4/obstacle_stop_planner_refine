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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "obstacle_stop_planner/obstacle_stop_planner.hpp"

namespace {
  template<typename T>
  T declare_parameter(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
    const std::string & name,
    const T & value)
  {
    return node.declare_parameter(name, rclcpp::ParameterValue(value)).get<T>();
  }
}

namespace obstacle_stop_planner
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using DynamicObjectArray = autoware_perception_msgs::msg::DynamicObjectArray;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using StopReasonArray = autoware_planning_msgs::msg::StopReasonArray;
using Float32MultiArrayStamped = autoware_debug_msgs::msg::Float32MultiArrayStamped;
using Point3d = autoware_utils::Point3d;

class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  explicit ObstacleStopPlannerNode(const rclcpp::NodeOptions & options);

private:
  // Publisher
  rclcpp::Publisher<Trajectory>::SharedPtr pub_path_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_viz_;
  rclcpp::Publisher<StopReasonArray>::SharedPtr pub_stop_reason_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_acc_debug_;


  // Subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr sub_path_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_obstacle_pointcloud_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_current_velocity_;
  rclcpp::Subscription<DynamicObjectArray>::SharedPtr sub_dynamic_object_;

  autoware_utils::SelfPoseListener self_pose_listener_;
  TransformListener transform_listener_;

  PointCloud2::ConstSharedPtr obstacle_pointcloud_;
  TwistStamped::ConstSharedPtr current_velocity_;
  DynamicObjectArray::ConstSharedPtr object_array_;

  std::unique_ptr<obstacle_stop_planner::ObstacleStopPlanner> planner_;

  bool isDataReady();
  Input createInputData(const Trajectory & trajectory);
  void onTrajectory(const Trajectory::ConstSharedPtr input_msg);
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
