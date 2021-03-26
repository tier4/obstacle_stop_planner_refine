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

#include <map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "obstacle_stop_planner/obstacle_point_cloud.hpp"
#include "obstacle_stop_planner/vehicle.hpp"
#include "obstacle_stop_planner/trajectory.hpp"

namespace motion_planning
{
class ObstacleStopPlannerNode : public rclcpp::Node
{
public:
  ObstacleStopPlannerNode();

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
  rclcpp::Subscription<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr expand_stop_range_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;

  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // ObstacleStopPlanner impl_;
  ObstaclePointCloud obstacle_pointcloud_;
  VehicleInfo vehicle_info_;
  Trajectory trajectory_;

  /*
   * Parameter
   */
  std::unique_ptr<motion_planning::AdaptiveCruiseController> acc_controller_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr_;
  rclcpp::Time prev_col_point_time_;
  pcl::PointXYZ prev_col_point_;
  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void pathCallback(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);
  void externalExpandStopRangeCallback(
    const autoware_debug_msgs::msg::Float32Stamped::ConstSharedPtr input_msg);

private:
  bool convexHull(
    const std::vector<cv::Point2d> pointcloud, std::vector<cv::Point2d> & polygon_points);
  void createOneStepPolygon(
    const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
    std::vector<cv::Point2d> & polygon, const double expand_width = 0.0);
  bool getSelfPose(
    const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
    geometry_msgs::msg::Pose & self_pose);
  void insertSlowDownVelocity(
    const size_t slow_down_start_point_idx, const double slow_down_target_vel, double slow_down_vel,
    autoware_planning_msgs::msg::Trajectory & output_path);

  double calcSlowDownTargetVel(const double lateral_deviation);
  bool extendTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory,
    const double extend_distance,
    autoware_planning_msgs::msg::Trajectory & output_trajectory);

  autoware_planning_msgs::msg::TrajectoryPoint getExtendTrajectoryPoint(
    double extend_distance, const autoware_planning_msgs::msg::TrajectoryPoint & goal_point);
};
}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
