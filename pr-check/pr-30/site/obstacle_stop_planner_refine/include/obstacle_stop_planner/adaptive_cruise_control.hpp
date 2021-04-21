// Copyright 2020 Tier IV, Inc.
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

#ifndef OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_
#define OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_

#include <vector>
#include <tuple>
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "boost/optional.hpp"

#include "autoware_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

namespace obstacle_stop_planner
{
using autoware_utils::Point2d;
using autoware_utils::Point3d;
using autoware_utils::Polygon2d;
using autoware_utils::Polygon3d;
using boost::optional;

namespace adaptive_cruise_controller
{
struct Input
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  size_t nearest_collision_point_idx;
  geometry_msgs::msg::Pose self_pose;
  Point2d nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  autoware_perception_msgs::msg::DynamicObjectArray object_array;
  geometry_msgs::msg::TwistStamped current_velocity;
};
}  // namespace adaptive_cruise_controller

class AdaptiveCruiseController
{
public:
  AdaptiveCruiseController(
    const rclcpp::node_interfaces::NodeLoggingInterface::ConstSharedPtr node_logging,
    const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
    const std::shared_ptr<vehicle_info_util::VehicleInfo> & vehicle_info,
    const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param);

  void updateParameter(const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param);

  boost::optional<autoware_planning_msgs::msg::Trajectory> insertAdaptiveCruiseVelocity(
    const adaptive_cruise_controller::Input & input);

  autoware_debug_msgs::msg::Float32MultiArrayStamped getDebugMsg() {return debug_values_;}

private:
  // rclcpp::Publisher<autoware_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr pub_debug_;

  rclcpp::node_interfaces::NodeLoggingInterface::ConstSharedPtr node_logging_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;

  /*
   * Parameter
   */
  std::shared_ptr<vehicle_info_util::VehicleInfo> vehicle_info_;
  std::shared_ptr<AdaptiveCruiseControlParameter> param_;

  rclcpp::Time prev_collision_point_time_;
  Point2d prev_collision_point_;
  double prev_target_vehicle_time_ = 0.0;
  double prev_target_vehicle_dist_ = 0.0;
  double prev_target_velocity_ = 0.0;
  bool prev_collision_point_valid_ = false;
  std::vector<geometry_msgs::msg::TwistStamped> est_vel_que_;
  double prev_upper_velocity_ = 0.0;
  double min_behavior_stop_margin_ = 0.0;

  double getMedianVel(const std::vector<geometry_msgs::msg::TwistStamped> & vel_que);
  static double lowpass_filter(
    const double current_value, const double prev_value,
    const double gain);
  double calcDistanceToNearestPointOnPath(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const int nearest_point_idx,
    const geometry_msgs::msg::Pose & self_pose, const Point2d & nearest_collision_point,
    const rclcpp::Time & nearest_collision_point_time);
  static double calcTrajYaw(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const int collision_point_idx);
  optional<double> estimatePointVelocityFromObject(
    const autoware_perception_msgs::msg::DynamicObjectArray & object_array,
    const double traj_yaw,
    const Point2d & nearest_collision_point);
  optional<double> estimatePointVelocityFromPcl(
    const double traj_yaw, const Point2d & nearest_collision_point,
    const rclcpp::Time & nearest_collision_point_time);
  double estimateRoughPointVelocity(const double current_vel);
  double calcUpperVelocity(const double dist_to_col, const double obj_vel, const double self_vel);
  double calcThreshDistToForwardObstacle(const double current_vel, const double obj_vel) const;
  double calcBaseDistToForwardObstacle(const double current_vel, const double obj_vel) const;
  double calcTargetVelocity_P(const double target_dist, const double current_dist) const;
  static double calcTargetVelocity_I(const double target_dist, const double current_dist);
  double calcTargetVelocity_D(const double target_dist, const double current_dist);
  double calcTargetVelocityByPID(
    const double current_vel, const double current_dist, const double obj_vel);

  autoware_planning_msgs::msg::Trajectory insertMaxVelocityToPath(
    const geometry_msgs::msg::Pose & /*self_pose*/,
    const double current_vel,
    const double target_vel,
    const double dist_to_collision_point,
    const autoware_planning_msgs::msg::Trajectory & input_trajectory) const;
  void registerQueToVelocity(const double vel, const rclcpp::Time & vel_time);

  /* Debug */
  autoware_debug_msgs::msg::Float32MultiArrayStamped debug_values_;
  enum DBGVAL
  {
    ESTIMATED_VEL_PCL = 0,
    ESTIMATED_VEL_OBJ = 1,
    ESTIMATED_VEL_FINAL = 2,
    FORWARD_OBJ_DISTANCE = 3,
    CURRENT_VEL = 4,
    UPPER_VEL_P = 5,
    UPPER_VEL_I = 6,
    UPPER_VEL_D = 7,
    UPPER_VEL_RAW = 8,
    UPPER_VEL = 9
  };
  static constexpr unsigned int num_debug_values_ = 10;
};

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_
