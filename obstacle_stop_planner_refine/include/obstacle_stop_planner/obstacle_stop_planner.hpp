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
#include "obstacle_stop_planner/util.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"

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
  size_t segment_index;
  Point2d obstacle_point;
};

class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlanner
{
public:
  ObstacleStopPlanner(
    const rclcpp::node_interfaces::NodeLoggingInterface::ConstSharedPtr node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::shared_ptr<vehicle_info_util::VehicleInfo> & vehicle_info,
    const std::shared_ptr<StopControlParameter> & stop_param,
    const std::shared_ptr<SlowDownControlParameter> & slow_down_param,
    const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param);

  // Update Parameters
  void updateParameters(
    const std::shared_ptr<StopControlParameter> & stop_param,
    const std::shared_ptr<SlowDownControlParameter> & slow_down_param,
    const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param);

  Output processTrajectory(const Input & input);

private:
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  std::unique_ptr<obstacle_stop_planner::AdaptiveCruiseController> acc_controller_;

  /*
   * Parameter
   */
  std::shared_ptr<vehicle_info_util::VehicleInfo> vehicle_info_;
  std::shared_ptr<StopControlParameter> stop_param_;
  std::shared_ptr<SlowDownControlParameter> slow_down_param_;
  std::shared_ptr<AdaptiveCruiseControlParameter> acc_param_;

public:
  double search_radius_ = 0.0;

  std::vector<Point3d> searchCandidateObstacle(
    const Trajectory & trajectory, const std::vector<Point3d> & obstacle_pointcloud);

  boost::optional<Collision> findCollisionPoint(
    const Trajectory & trajectory, const std::vector<Point3d> & obstacle_points);

  boost::optional<Collision> findSlowDownPoint(
    const Trajectory & trajectory, const std::vector<Point3d> & obstacle_points);

  std::vector<LinearRing2d> createStopFootprints(const Trajectory & trajectory);

  std::vector<LinearRing2d> createSlowDownFootprints(const Trajectory & trajectory);

  std::vector<LinearRing2d> createVehicleFootprints(
    const Trajectory & trajectory,
    const double margin);

  std::vector<Polygon2d> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints);

  Polygon2d createHullFromFootprints(
    const LinearRing2d & area1, const LinearRing2d & area2);

  boost::optional<Point3d> findCollisionParticle(
    const Polygon2d & area,
    const std::vector<Point3d> & obstacle_points,
    const Point2d & base_point);

  /**
  * @fn
  * @brief Create adaptive cruise trajectory
  * Follow front vehicle adaptively
  * @param input trajectory
  * @param collision collision point and index
  * @return Trajectory
  */
  boost::optional<Trajectory> planAdaptiveCruise(const Input & input, const Collision & collision);

  /**
  * @fn
  * @brief Create slow down trajectory
  * All velocity in trajectory set to minimum velcity after slow_down_index.
  * @param input trajectory
  * @param collision collision point and index
  * @param obstacles associated points from pointcloud
  * @return Trajectory
  */
  Trajectory planSlowDown(
    const Trajectory & trajectory, const Collision & collision,
    const std::vector<Point3d> & obstacles);

  bool findFrontObstacles(
    const autoware_planning_msgs::msg::TrajectoryPoint & point,
    const std::vector<Point3d> & obstacles);

  /**
  * @brief Create Stop trajectory
  * All velocity in trajectory set to 0 after stop_index.
  * @param trajectory input trajectory
  * @param collision collision point and index
  * @return Trajectory
  */
  Trajectory planObstacleStop(const Trajectory & trajectory, const Collision & collision);

  double calcSlowDownTargetVel(const double lateral_deviation) const;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__OBSTACLE_STOP_PLANNER_HPP_
