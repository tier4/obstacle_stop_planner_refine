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
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"

namespace {
  obstacle_stop_planner::StopControlParameter createStopParameter(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
  {
    obstacle_stop_planner::StopControlParameter stop_param;
    stop_param.stop_margin = declare_parameter(node, "stop_planner.stop_margin", 5.0);
    stop_param.min_behavior_stop_margin = declare_parameter(node, "stop_planner.min_behavior_stop_margin", 2.0);
    stop_param.step_length = declare_parameter(node, "stop_planner.step_length", 1.0);
    stop_param.extend_distance = declare_parameter(node, "stop_planner.extend_distance", 0.0);
    stop_param.expand_stop_range = declare_parameter(node, "stop_planner.expand_stop_range", 0.0);

    return stop_param;
  }

  obstacle_stop_planner::SlowDownControlParameter createSlowDownParameter(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
  {
    obstacle_stop_planner::SlowDownControlParameter slow_down_param;
    slow_down_param.slow_down_margin = declare_parameter(node, "slow_down_planner.slow_down_margin", 5.0);
    slow_down_param.expand_slow_down_range =
      declare_parameter(node, "slow_down_planner.expand_slow_down_range", 1.0);
    slow_down_param.max_slow_down_vel = declare_parameter(node, "slow_down_planner.max_slow_down_vel", 4.0);
    slow_down_param.min_slow_down_vel = declare_parameter(node, "slow_down_planner.min_slow_down_vel", 2.0);
    slow_down_param.max_deceleration = declare_parameter(node, "slow_down_planner.max_deceleration", 2.0);
    slow_down_param.enable_slow_down = declare_parameter(node, "enable_slow_down", false);

    return slow_down_param;
  }

  obstacle_stop_planner::AdaptiveCruiseControlParameter createAccParameter(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
  {
    obstacle_stop_planner::AdaptiveCruiseControlParameter acc_param;
    const std::string acc_ns = "adaptive_cruise_control.";
    acc_param.use_object_to_est_vel =
      declare_parameter(node, acc_ns + "use_object_to_estimate_vel", true);
    acc_param.use_pcl_to_est_vel = declare_parameter(node, acc_ns + "use_pcl_to_estimate_vel", true);
    acc_param.consider_obj_velocity = declare_parameter(node, acc_ns + "consider_obj_velocity", true);
    acc_param.obstacle_stop_velocity_thresh =
      declare_parameter(node, acc_ns + "obstacle_stop_velocity_thresh", 1.0);
    acc_param.emergency_stop_acceleration =
      declare_parameter(node, acc_ns + "emergency_stop_acceleration", -3.5);
    acc_param.obstacle_emergency_stop_acceleration =
      declare_parameter(node, acc_ns + "obstacle_emergency_stop_acceleration", -5.0);
    acc_param.emergency_stop_idling_time =
      declare_parameter(node, acc_ns + "emergency_stop_idling_time", 0.5);
    acc_param.min_dist_stop = declare_parameter(node, acc_ns + "min_dist_stop", 4.0);
    acc_param.max_standard_acceleration =
      declare_parameter(node, acc_ns + "max_standard_acceleration", 0.5);
    acc_param.min_standard_acceleration =
      declare_parameter(node, acc_ns + "min_standard_acceleration", -1.0);
    acc_param.standard_idling_time = declare_parameter(node, acc_ns + "standard_idling_time", 0.5);
    acc_param.min_dist_standard = declare_parameter(node, acc_ns + "min_dist_standard", 4.0);
    acc_param.obstacle_min_standard_acceleration =
      declare_parameter(node, acc_ns + "obstacle_min_standard_acceleration", -1.5);
    acc_param.margin_rate_to_change_vel =
      declare_parameter(node, acc_ns + "margin_rate_to_change_vel", 0.3);
    acc_param.use_time_compensation_to_dist =
      declare_parameter(node, acc_ns + "use_time_compensation_to_calc_distance", true);
    acc_param.lowpass_gain_ = declare_parameter(node, acc_ns + "lowpass_gain_of_upper_velocity", 0.6);

    /* parameter for pid in acc */
    acc_param.p_coeff_pos = declare_parameter(node, acc_ns + "p_coefficient_positive", 0.1);
    acc_param.p_coeff_neg = declare_parameter(node, acc_ns + "p_coefficient_negative", 0.3);
    acc_param.d_coeff_pos = declare_parameter(node, acc_ns + "d_coefficient_positive", 0.0);
    acc_param.d_coeff_neg = declare_parameter(node, acc_ns + "d_coefficient_negative", 0.1);

    /* parameter for speed estimation of obstacle */
    acc_param.object_polygon_length_margin =
      declare_parameter(node, acc_ns + "object_polygon_length_margin", 2.0);
    acc_param.object_polygon_width_margin =
      declare_parameter(node, acc_ns + "object_polygon_width_margin", 0.5);
    acc_param.valid_est_vel_diff_time =
      declare_parameter(node, acc_ns + "valid_estimated_vel_diff_time", 1.0);
    acc_param.valid_vel_que_time = declare_parameter(node, acc_ns + "valid_vel_que_time", 0.5);
    acc_param.valid_est_vel_max = declare_parameter(node, acc_ns + "valid_estimated_vel_max", 20.0);
    acc_param.valid_est_vel_min = declare_parameter(node, acc_ns + "valid_estimated_vel_min", -20.0);
    acc_param.thresh_vel_to_stop = declare_parameter(node, acc_ns + "thresh_vel_to_stop", 0.5);
    acc_param.use_rough_est_vel =
      declare_parameter(node, acc_ns + "use_rough_velocity_estimation", false);
    acc_param.rough_velocity_rate = declare_parameter(node, acc_ns + "rough_velocity_rate", 0.9);

    return acc_param;
  }
}

namespace obstacle_stop_planner
{
ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node{"obstacle_stop_planner", node_options},
  self_pose_listener_(this)
{
  // Parameters
  auto parameter_interface = this->get_node_parameters_interface();
  const auto stop_param = createStopParameter(parameter_interface);
  const auto slow_down_param = createSlowDownParameter(parameter_interface);
  const auto acc_param = createStopParameter(parameter_interface);

  // Vehicle Info
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  planner_ = std::make_unique<obstacle_stop_planner::ObstacleStopPlanner>(
    this,
    vehicle_info,
    stop_param,
    slow_down_param,
    acc_param);

  // Publishers
  pub_path_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_debug_viz_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  pub_stop_reason_ = create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
  pub_acc_debug_ = create_publisher<Float32MultiArrayStamped>("~/debug_values", 1);

  // Subscribers
  sub_obstacle_pointcloud_ = create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    [this](const PointCloud2::ConstSharedPtr msg) {obstacle_pointcloud_ = msg;});
  sub_current_velocity_ = create_subscription<TwistStamped>(
    "~/input/twist", 1,
    [this](const TwistStamped::ConstSharedPtr msg) {current_velocity_ = msg;});
  sub_dynamic_object_ =
    create_subscription<DynamicObjectArray>(
    "~/input/objects", 1,
    [this](const DynamicObjectArray::ConstSharedPtr msg) {object_array_ = msg;});
  sub_path_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::onTrajectory, this, std::placeholders::_1));
}

bool ObstacleStopPlannerNode::isDataReady()
{
  if (!obstacle_pointcloud_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return false;
  }

  if (!current_velocity_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return false;
  }

  return true;
}

void ObstacleStopPlannerNode::onTrajectory(
  const Trajectory::ConstSharedPtr input_msg)
{
  if(!isDataReady()) {
    return;
  }

  Input input;
  input.current_pose = self_pose_listener_.getCurrentPose()->pose;
  input.current_velocity = *current_velocity_;
  input.input_trajectory = *input_msg;
  input.object_array = *object_array_;
  input.obstacle_pointcloud = *obstacle_pointcloud_;

  const auto output = planner_->processTrajectory(input);

  pub_path_->publish(output.output_trajectory);
  pub_debug_viz_->publish(output.debug_viz_msg);
  pub_stop_reason_->publish(output.stop_reason);
  pub_acc_debug_->publish(output.acc_debug_msg);
}
}  // namespace obstacle_stop_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  obstacle_stop_planner::ObstacleStopPlannerNode)
