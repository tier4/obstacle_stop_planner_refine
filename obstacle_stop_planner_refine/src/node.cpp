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

#include <memory>
#include <string>

#include "obstacle_stop_planner/node.hpp"
#include "vehicle_info_util/vehicle_info.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"

namespace obstacle_stop_planner
{
ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node{"obstacle_stop_planner", node_options},
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  pointcloud_received_(false),
  current_velocity_reveived_(false)
{
  // Vehicle Info
  auto i = vehicle_info_util::VehicleInfo::create(*this);

  // Parameters
  obstacle_stop_planner::StopControlParameter stop_param;
  stop_param.stop_margin = declare_parameter("stop_planner.stop_margin", 5.0);
  stop_param.min_behavior_stop_margin =
    declare_parameter("stop_planner.min_behavior_stop_margin", 2.0);
  stop_param.step_length = declare_parameter("stop_planner.step_length", 1.0);
  stop_param.extend_distance = declare_parameter("stop_planner.extend_distance", 0.0);
  stop_param.expand_stop_range = declare_parameter("stop_planner.expand_stop_range", 0.0);

  obstacle_stop_planner::SlowDownControlParameter slow_down_param;
  slow_down_param.slow_down_margin = declare_parameter("slow_down_planner.slow_down_margin", 5.0);
  slow_down_param.expand_slow_down_range =
    declare_parameter("slow_down_planner.expand_slow_down_range", 1.0);
  slow_down_param.max_slow_down_vel = declare_parameter("slow_down_planner.max_slow_down_vel", 4.0);
  slow_down_param.min_slow_down_vel = declare_parameter("slow_down_planner.min_slow_down_vel", 2.0);
  slow_down_param.max_deceleration = declare_parameter("slow_down_planner.max_deceleration", 2.0);
  slow_down_param.enable_slow_down = declare_parameter("enable_slow_down", false);

  stop_param.stop_margin += i.wheel_base_m_ + i.front_overhang_m_;
  stop_param.min_behavior_stop_margin +=
    i.wheel_base_m_ + i.front_overhang_m_;
  slow_down_param.slow_down_margin += i.wheel_base_m_ + i.front_overhang_m_;
  stop_param.stop_search_radius = stop_param.step_length + std::hypot(
    i.vehicle_width_m_ / 2.0 + stop_param.expand_stop_range,
    i.vehicle_length_m_ / 2.0);
  slow_down_param.slow_down_search_radius = stop_param.step_length + std::hypot(
    i.vehicle_width_m_ / 2.0 + slow_down_param.expand_slow_down_range,
    i.vehicle_length_m_ / 2.0);

  // Parameters for adaptive_cruise_control
  obstacle_stop_planner::AdaptiveCruiseControlParameter acc_param;
  std::string acc_ns = "adaptive_cruise_control.";
  acc_param.use_object_to_est_vel =
    declare_parameter(acc_ns + "use_object_to_estimate_vel", true);
  acc_param.use_pcl_to_est_vel = declare_parameter(acc_ns + "use_pcl_to_estimate_vel", true);
  acc_param.consider_obj_velocity = declare_parameter(acc_ns + "consider_obj_velocity", true);
  acc_param.obstacle_stop_velocity_thresh =
    declare_parameter(acc_ns + "obstacle_stop_velocity_thresh", 1.0);
  acc_param.emergency_stop_acceleration =
    declare_parameter(acc_ns + "emergency_stop_acceleration", -3.5);
  acc_param.obstacle_emergency_stop_acceleration =
    declare_parameter(acc_ns + "obstacle_emergency_stop_acceleration", -5.0);
  acc_param.emergency_stop_idling_time =
    declare_parameter(acc_ns + "emergency_stop_idling_time", 0.5);
  acc_param.min_dist_stop = declare_parameter(acc_ns + "min_dist_stop", 4.0);
  acc_param.max_standard_acceleration =
    declare_parameter(acc_ns + "max_standard_acceleration", 0.5);
  acc_param.min_standard_acceleration =
    declare_parameter(acc_ns + "min_standard_acceleration", -1.0);
  acc_param.standard_idling_time = declare_parameter(acc_ns + "standard_idling_time", 0.5);
  acc_param.min_dist_standard = declare_parameter(acc_ns + "min_dist_standard", 4.0);
  acc_param.obstacle_min_standard_acceleration =
    declare_parameter(acc_ns + "obstacle_min_standard_acceleration", -1.5);
  acc_param.margin_rate_to_change_vel =
    declare_parameter(acc_ns + "margin_rate_to_change_vel", 0.3);
  acc_param.use_time_compensation_to_dist =
    declare_parameter(acc_ns + "use_time_compensation_to_calc_distance", true);
  acc_param.lowpass_gain_ = declare_parameter(acc_ns + "lowpass_gain_of_upper_velocity", 0.6);

  /* parameter for pid in acc */
  acc_param.p_coeff_pos = declare_parameter(acc_ns + "p_coefficient_positive", 0.1);
  acc_param.p_coeff_neg = declare_parameter(acc_ns + "p_coefficient_negative", 0.3);
  acc_param.d_coeff_pos = declare_parameter(acc_ns + "d_coefficient_positive", 0.0);
  acc_param.d_coeff_neg = declare_parameter(acc_ns + "d_coefficient_negative", 0.1);

  /* parameter for speed estimation of obstacle */
  acc_param.object_polygon_length_margin =
    declare_parameter(acc_ns + "object_polygon_length_margin", 2.0);
  acc_param.object_polygon_width_margin =
    declare_parameter(acc_ns + "object_polygon_width_margin", 0.5);
  acc_param.valid_est_vel_diff_time =
    declare_parameter(acc_ns + "valid_estimated_vel_diff_time", 1.0);
  acc_param.valid_vel_que_time = declare_parameter(acc_ns + "valid_vel_que_time", 0.5);
  acc_param.valid_est_vel_max = declare_parameter(acc_ns + "valid_estimated_vel_max", 20.0);
  acc_param.valid_est_vel_min = declare_parameter(acc_ns + "valid_estimated_vel_min", -20.0);
  acc_param.thresh_vel_to_stop = declare_parameter(acc_ns + "thresh_vel_to_stop", 0.5);
  acc_param.use_rough_est_vel =
    declare_parameter(acc_ns + "use_rough_velocity_estimation", false);
  acc_param.rough_velocity_rate = declare_parameter(acc_ns + "rough_velocity_rate", 0.9);

  planner_ = std::make_unique<obstacle_stop_planner::ObstacleStopPlanner>(
    this,
    i,
    stop_param,
    slow_down_param,
    acc_param);

  // Publishers
  path_pub_ =
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "~/input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1));
  expand_stop_range_sub_ = this->create_subscription<autoware_planning_msgs::msg::ExpandStopRange>(
    "~/input/expand_stop_range", 1,
    std::bind(
      &ObstacleStopPlannerNode::externalExpandStopRangeCallback, this, std::placeholders::_1));
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  planner_->obstaclePointcloudCallback(input_msg);
  pointcloud_received_ = true;
}

void ObstacleStopPlannerNode::pathCallback(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  if (!pointcloud_received_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_reveived_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  const auto output_msg = planner_->pathCallback(input_msg, tf_buffer_);
  path_pub_->publish(output_msg);
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg)
{
  planner_->externalExpandStopRangeCallback(input_msg);
}

void ObstacleStopPlannerNode::dynamicObjectCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  planner_->dynamicObjectCallback(input_msg);
}

void ObstacleStopPlannerNode::currentVelocityCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg)
{
  planner_->currentVelocityCallback(input_msg);
  current_velocity_reveived_ = true;
}
}  // namespace obstacle_stop_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  obstacle_stop_planner::ObstacleStopPlannerNode)
