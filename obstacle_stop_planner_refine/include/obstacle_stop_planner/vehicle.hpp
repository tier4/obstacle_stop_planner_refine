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

#ifndef OBSTACLE_STOP_PLANNER__VEHICLE_HPP_
#define OBSTACLE_STOP_PLANNER__VEHICLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_util/vehicle_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle_stop_planner/util.hpp"

namespace obstacle_stop_planner
{

class VehicleInfo : public vehicle_info_util::VehicleInfo
{
public:
  VehicleInfo(
    vehicle_info_util::VehicleInfo super,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter)
  : vehicle_info_util::VehicleInfo(super)
  {
    // Parameters
    stop_margin_ =
      parameter->declare_parameter(
      "stop_planner.stop_margin",
      rclcpp::ParameterValue(5.0)).get<double>();
    min_behavior_stop_margin_ = parameter->declare_parameter(
      "stop_planner.min_behavior_stop_margin", rclcpp::ParameterValue(2.0)).get<double>();
    step_length_ =
      parameter->declare_parameter(
      "stop_planner.step_length",
      rclcpp::ParameterValue(1.0)).get<double>();
    extend_distance_ = parameter->declare_parameter(
      "stop_planner.extend_distance", rclcpp::ParameterValue(
        0.0)).get<double>();
    expand_stop_range_ = parameter->declare_parameter(
      "stop_planner.expand_stop_range", rclcpp::ParameterValue(
        0.0)).get<double>();

    slow_down_margin_ = parameter->declare_parameter(
      "slow_down_planner.slow_down_margin", rclcpp::ParameterValue(
        5.0)).get<double>();
    expand_slow_down_range_ = parameter->declare_parameter(
      "slow_down_planner.expand_slow_down_range", rclcpp::ParameterValue(1.0)).get<double>();
    max_slow_down_vel_ = parameter->declare_parameter(
      "slow_down_planner.max_slow_down_vel", rclcpp::ParameterValue(
        4.0)).get<double>();
    min_slow_down_vel_ = parameter->declare_parameter(
      "slow_down_planner.min_slow_down_vel", rclcpp::ParameterValue(
        2.0)).get<double>();
    max_deceleration_ = parameter->declare_parameter(
      "slow_down_planner.max_deceleration", rclcpp::ParameterValue(
        2.0)).get<double>();
    enable_slow_down_ =
      parameter->declare_parameter("enable_slow_down", rclcpp::ParameterValue(false)).get<bool>();

    stop_margin_ += wheel_base_m_ + front_overhang_m_;
    min_behavior_stop_margin_ += wheel_base_m_ + front_overhang_m_;
    slow_down_margin_ += wheel_base_m_ + front_overhang_m_;
    stop_search_radius_ =
      step_length_ +
      std::hypot(vehicle_width_m_ / 2.0 + expand_stop_range_, vehicle_length_m_ / 2.0);
    slow_down_search_radius_ =
      step_length_ +
      std::hypot(vehicle_width_m_ / 2.0 + expand_slow_down_range_, vehicle_length_m_ / 2.0);
  }

  geometry_msgs::msg::Pose getVehicleCenterFromBase(
    const geometry_msgs::msg::Pose & base_pose) const
  {
    geometry_msgs::msg::Pose center_pose;
    const double yaw = getYawFromGeometryMsgsQuaternion(base_pose.orientation);
    center_pose.position.x =
      base_pose.position.x + (vehicle_length_m_ / 2.0 - rear_overhang_m_) * std::cos(yaw);
    center_pose.position.y =
      base_pose.position.y + (vehicle_length_m_ / 2.0 - rear_overhang_m_) * std::sin(yaw);
    center_pose.position.z = base_pose.position.z;
    center_pose.orientation = base_pose.orientation;
    return center_pose;
  }

  double getSearchRadius() const
  {
    if (enable_slow_down_) {
      return slow_down_search_radius_;
    } else {
      return stop_search_radius_;
    }
  }

  double stop_margin_;
  double min_behavior_stop_margin_;
  double step_length_;
  double extend_distance_;
  double expand_stop_range_;
  double slow_down_margin_;
  double expand_slow_down_range_;
  double max_slow_down_vel_;
  double min_slow_down_vel_;
  double max_deceleration_;
  bool enable_slow_down_;
  double stop_search_radius_;
  double slow_down_search_radius_;

private:
};

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__VEHICLE_HPP_
