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
#ifndef OBSTACLE_STOP_PLANNER__OBSTACLE_STOP_PLANNER_HPP_
#define OBSTACLE_STOP_PLANNER__OBSTACLE_STOP_PLANNER_HPP_

#include <map>
#include <memory>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#include "obstacle_stop_planner/visibility_control.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/expand_stop_range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "obstacle_stop_planner/obstacle_point_cloud.hpp"
#include "obstacle_stop_planner/point_helper.hpp"
#include "obstacle_stop_planner/util.hpp"

namespace obstacle_stop_planner
{
class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlanner
{
public:
  ObstacleStopPlanner(
    rclcpp::Node * node,
    const vehicle_info_util::VehicleInfo & vehicle_info,
    const StopControlParameter & stop_param,
    const SlowDownControlParameter & slow_down_param,
    const AdaptiveCruiseControlParameter & acc_param);

  void obstaclePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  autoware_planning_msgs::msg::Trajectory
  pathCallback(
    const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg,
    tf2_ros::Buffer & tf_buffer);
  void dynamicObjectCallback(
    const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg);
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg);
  void externalExpandStopRangeCallback(
    const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg);

private:
  rclcpp::Node * node_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  StopControlParameter stop_param_;
  SlowDownControlParameter slow_down_param_;
  AdaptiveCruiseControlParameter acc_param_;

  /*
   * Parameter
   */
  std::unique_ptr<obstacle_stop_planner::AdaptiveCruiseController> acc_controller_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr object_ptr_;
  rclcpp::Time prev_col_point_time_;
  pcl::PointXYZ prev_col_point_;

private:
  geometry_msgs::msg::Pose getSelfPose(
    const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer);
  autoware_planning_msgs::msg::Trajectory insertSlowDownVelocity(
    const size_t slow_down_start_point_idx,
    const double slow_down_target_vel,
    const double slow_down_vel,
    const autoware_planning_msgs::msg::Trajectory & input_path);
  double calcSlowDownTargetVel(const double lateral_deviation) const;
  static std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr> getSlowDownPointcloud(
    const bool is_slow_down, const bool enable_slow_down,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud,
    const Point2d & prev_center_point,
    const Point2d & next_center_point,
    const double search_radius,
    const Polygon2d & one_step_polygon,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
    const bool candidate_slow_down);
  std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr> getCollisionPointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
    const Point2d & prev_center_point,
    const Point2d & next_center_point,
    const double search_radius, const Polygon2d & one_step_polygon,
    const autoware_planning_msgs::msg::TrajectoryPoint & trajectory_point,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud,
    const bool is_collision);
  autoware_planning_msgs::msg::Trajectory insertStopPoint(
    const size_t search_start_index,
    const autoware_planning_msgs::msg::Trajectory & base_path,
    const Point2d & nearest_collision_point,
    const autoware_planning_msgs::msg::Trajectory & input_msg);
  autoware_planning_msgs::msg::Trajectory insertSlowDownPoint(
    const size_t search_start_index,
    const autoware_planning_msgs::msg::Trajectory & base_path,
    const Point2d & nearest_slow_down_point,
    const double slow_down_target_vel,
    const double slow_down_margin,
    const autoware_planning_msgs::msg::Trajectory & input_msg);
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_STOP_PLANNER_HPP_
