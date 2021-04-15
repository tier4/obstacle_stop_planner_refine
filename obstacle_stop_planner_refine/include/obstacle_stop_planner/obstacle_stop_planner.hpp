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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "obstacle_stop_planner/adaptive_cruise_control.hpp"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "obstacle_stop_planner/obstacle_point_cloud.hpp"
#include "obstacle_stop_planner/point_helper.hpp"
#include "obstacle_stop_planner/util.hpp"

namespace obstacle_stop_planner
{
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using Point3d = autoware_utils::Point3d;
using LinearRing2d = autoware_utils::LinearRing2d;
struct Input
{
  geometry_msgs::msg::Pose current_pose;
  std::vector<Point3d> obstacle_pointcloud;
  rclcpp::Time pointcloud_header_time;
  geometry_msgs::msg::TwistStamped current_velocity;
  autoware_perception_msgs::msg::DynamicObjectArray object_array;
  Trajectory input_trajectory;
};

struct Output
{
  Trajectory output_trajectory;
  visualization_msgs::msg::MarkerArray debug_viz_msg;
  autoware_planning_msgs::msg::StopReasonArray stop_reason;
  autoware_debug_msgs::msg::Float32MultiArrayStamped acc_debug_msg;
};

struct Collision
{
  size_t trajectory_index;
  Point2d obstacle_point;
};

class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlanner
{
public:
  ObstacleStopPlanner(
    rclcpp::Node * node,
    const vehicle_info_util::VehicleInfo & vehicle_info,
    const StopControlParameter & stop_param,
    const SlowDownControlParameter & slow_down_param,
    const AdaptiveCruiseControlParameter & acc_param);

  // Update Parameters
  void updateParameters(
    const StopControlParameter & stop_param,
    const SlowDownControlParameter & slow_down_param,
    const AdaptiveCruiseControlParameter & acc_param);

  Output processTrajectory(const Input & input);

private:
  rclcpp::Node * node_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  std::unique_ptr<obstacle_stop_planner::AdaptiveCruiseController> acc_controller_;

  /*
   * Parameter
   */
  vehicle_info_util::VehicleInfo vehicle_info_;
  StopControlParameter stop_param_;
  SlowDownControlParameter slow_down_param_;
  AdaptiveCruiseControlParameter acc_param_;

public:
  std::vector<Point3d> searchCandidateObstacle(
    const Trajectory & trajectory, const std::vector<Point3d> & obstacle_pointcloud);

  boost::optional<Collision> findCollisionPoint(
    const Trajectory & trajectory, const std::vector<Point3d> & obstacle_points);

  std::vector<LinearRing2d> createVehicleFootprints(const Trajectory & trajectory);

  std::vector<Polygon2d> createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints);

  Polygon2d createHullFromFootprints(
  const LinearRing2d & area1, const LinearRing2d & area2);

  boost::optional<Point3d> findCollisionParticle(const Polygon2d & area, const std::vector<Point3d> & obstacle_points, const Point2d & base_point);

  boost::optional<Trajectory> adaptiveCruise(const Input & input, const Collision & collision);

  Trajectory slowDown(const Trajectory & trajectory, const Collision & collision);

  Trajectory obstacleStop(const Trajectory & trajectory, const Collision & collision);



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
  // autoware_planning_msgs::msg::Trajectory insertSlowDownPoint(
  //   const size_t search_start_index,
  //   const autoware_planning_msgs::msg::Trajectory & base_path,
  //   const Point2d & nearest_slow_down_point,
  //   const double slow_down_target_vel,
  //   const double slow_down_margin,
  //   const autoware_planning_msgs::msg::Trajectory & input_msg);
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_STOP_PLANNER_HPP_
