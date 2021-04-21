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
#include <vector>

#include "obstacle_stop_planner/node.hpp"
#include "vehicle_info_util/vehicle_info.hpp"
#include "obstacle_stop_planner/obstacle_point_cloud.hpp"

namespace
{
obstacle_stop_planner::StopControlParameter createStopParameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
{
  obstacle_stop_planner::StopControlParameter stop_param;
  stop_param.stop_margin = declare_parameter(node, "stop_planner.stop_margin", 5.0);
  stop_param.min_behavior_stop_margin = declare_parameter(
    node,
    "stop_planner.min_behavior_stop_margin",
    2.0);
  stop_param.step_length = declare_parameter(node, "stop_planner.step_length", 1.0);
  stop_param.extend_distance = declare_parameter(node, "stop_planner.extend_distance", 0.0);
  stop_param.expand_stop_range = declare_parameter(node, "stop_planner.expand_stop_range", 0.0);

  return stop_param;
}

obstacle_stop_planner::SlowDownControlParameter createSlowDownParameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
{
  obstacle_stop_planner::SlowDownControlParameter slow_down_param;
  slow_down_param.slow_down_margin = declare_parameter(
    node, "slow_down_planner.slow_down_margin",
    5.0);
  slow_down_param.expand_slow_down_range =
    declare_parameter(node, "slow_down_planner.expand_slow_down_range", 1.0);
  slow_down_param.max_slow_down_vel = declare_parameter(
    node, "slow_down_planner.max_slow_down_vel",
    4.0);
  slow_down_param.min_slow_down_vel = declare_parameter(
    node, "slow_down_planner.min_slow_down_vel",
    2.0);
  slow_down_param.enable_slow_down = declare_parameter(node, "enable_slow_down", false);

  return slow_down_param;
}

obstacle_stop_planner::AdaptiveCruiseControlParameter createAccParameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node)
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
}  // namespace

namespace obstacle_stop_planner
{
ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node{"obstacle_stop_planner", node_options},
  self_pose_listener_(this),
  transform_listener_(this)
{
  // Parameters
  auto parameter_interface = this->get_node_parameters_interface();
  stop_param_ = std::make_shared<StopControlParameter>(createStopParameter(parameter_interface));
  slow_down_param_ =
    std::make_shared<SlowDownControlParameter>(createSlowDownParameter(parameter_interface));
  acc_param_ =
    std::make_shared<AdaptiveCruiseControlParameter>(createAccParameter(parameter_interface));

  // Vehicle Info
  const auto vehicle_info = std::make_shared<vehicle_info_util::VehicleInfo>(
    vehicle_info_util::VehicleInfo::create(
      *this));

  planner_ = std::make_unique<obstacle_stop_planner::ObstacleStopPlanner>(
    this->get_node_logging_interface(),
    this->get_node_clock_interface(),
    vehicle_info,
    stop_param_,
    slow_down_param_,
    acc_param_);

  // Publishers
  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker", 1);
  pub_stop_reason_ = create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
  pub_debug_acc_ = create_publisher<Float32MultiArrayStamped>("~/debug_values", 1);

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
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::onTrajectory, this, std::placeholders::_1));

  // Parameter Callback
  set_param_res_ =
    add_on_set_parameters_callback(
    std::bind(
      &ObstacleStopPlannerNode::onParameter, this,
      std::placeholders::_1));
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

  if (acc_param_->use_object_to_est_vel) {
    if (!object_array_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "waiting for objects...");
      return false;
    }
  }

  return true;
}

bool ObstacleStopPlannerNode::isTransformReady(const Trajectory & trajectory)
{
  // Check transform available
  if (!self_pose_listener_.getCurrentPose()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current pose...");
    return false;
  }

  const auto pointcloud = updatePointCloud(obstacle_pointcloud_);
  const auto transform = transform_listener_.getTransform(
    trajectory.header.frame_id,
    pointcloud->header.frame_id,
    pointcloud->header.stamp,
    rclcpp::Duration::from_seconds(0.5));

  if (!transform) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[obstacle_stop_planner] Failed to look up transform from " <<
        trajectory.header.frame_id << " to " << pointcloud->header.frame_id);
    return false;
  }

  return true;
}

Input ObstacleStopPlannerNode::createInputData(const Trajectory & trajectory)
{
  Input input;
  input.current_pose = self_pose_listener_.getCurrentPose()->pose;
  input.current_velocity = *current_velocity_;
  input.input_trajectory = trajectory;
  if (object_array_) {
    input.object_array = *object_array_;
  }
  input.pointcloud_header_time = obstacle_pointcloud_->header.stamp;

  // Transform pointcloud
  const auto pointcloud = updatePointCloud(obstacle_pointcloud_);
  const auto transform = transform_listener_.getTransform(
    trajectory.header.frame_id,
    pointcloud->header.frame_id,
    pointcloud->header.stamp,
    rclcpp::Duration::from_seconds(0.5));
  const auto transformed_pointcloud = transformObstacle(pointcloud, *transform);
  input.obstacle_pointcloud.reserve(transformed_pointcloud->points.size());
  for (const auto & point : transformed_pointcloud->points) {
    input.obstacle_pointcloud.emplace_back(Point3d {point.x, point.y, point.z});
  }

  return input;
}

void ObstacleStopPlannerNode::onTrajectory(
  const Trajectory::ConstSharedPtr input_msg)
{
  if (!isDataReady()) {
    return;
  }

  if (!isTransformReady(*input_msg)) {
    return;
  }

  const auto input = createInputData(*input_msg);

  const auto output = planner_->processTrajectory(input);

  pub_trajectory_->publish(output.output_trajectory);
  pub_debug_marker_array_->publish(output.debug_viz_msg);
  pub_stop_reason_->publish(output.stop_reason);
  pub_debug_acc_->publish(output.acc_debug_msg);
}

rcl_interfaces::msg::SetParametersResult ObstacleStopPlannerNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    update_parameter(parameters, "stop_planner.stop_margin", stop_param_->stop_margin);
    update_parameter(
      parameters, "stop_planner.min_behavior_stop_margin",
      stop_param_->min_behavior_stop_margin);
    update_parameter(parameters, "stop_planner.step_length", stop_param_->step_length);
    update_parameter(parameters, "stop_planner.extend_distance", stop_param_->extend_distance);
    update_parameter(parameters, "stop_planner.expand_stop_range", stop_param_->expand_stop_range);

    update_parameter(
      parameters, "slow_down_planner.slow_down_margin",
      slow_down_param_->slow_down_margin);
    update_parameter(
      parameters, "slow_down_planner.expand_slow_down_range",
      slow_down_param_->expand_slow_down_range);
    update_parameter(
      parameters, "slow_down_planner.max_slow_down_vel",
      slow_down_param_->max_slow_down_vel);
    update_parameter(
      parameters, "slow_down_planner.min_slow_down_vel",
      slow_down_param_->min_slow_down_vel);
    update_parameter(parameters, "enable_slow_down", slow_down_param_->enable_slow_down);

    const std::string acc_ns = "adaptive_cruise_control.";
    update_parameter(
      parameters, acc_ns + "use_object_to_estimate_vel",
      acc_param_->use_object_to_est_vel);
    update_parameter(
      parameters, acc_ns + "use_pcl_to_estimate_vel",
      acc_param_->use_pcl_to_est_vel);
    update_parameter(
      parameters, acc_ns + "consider_obj_velocity",
      acc_param_->consider_obj_velocity);
    update_parameter(
      parameters, acc_ns + "obstacle_stop_velocity_thresh",
      acc_param_->obstacle_stop_velocity_thresh);
    update_parameter(
      parameters, acc_ns + "emergency_stop_acceleration",
      acc_param_->emergency_stop_acceleration);
    update_parameter(
      parameters, acc_ns + "obstacle_emergency_stop_acceleration",
      acc_param_->obstacle_emergency_stop_acceleration);
    update_parameter(
      parameters, acc_ns + "emergency_stop_idling_time",
      acc_param_->emergency_stop_idling_time);
    update_parameter(parameters, acc_ns + "min_dist_stop", acc_param_->min_dist_stop);
    update_parameter(
      parameters, acc_ns + "max_standard_acceleration",
      acc_param_->max_standard_acceleration);
    update_parameter(
      parameters, acc_ns + "min_standard_acceleration",
      acc_param_->min_standard_acceleration);
    update_parameter(parameters, acc_ns + "standard_idling_time", acc_param_->standard_idling_time);
    update_parameter(parameters, acc_ns + "min_dist_standard", acc_param_->min_dist_standard);
    update_parameter(
      parameters, acc_ns + "obstacle_min_standard_acceleration",
      acc_param_->obstacle_min_standard_acceleration);
    update_parameter(
      parameters, acc_ns + "margin_rate_to_change_vel",
      acc_param_->margin_rate_to_change_vel);
    update_parameter(
      parameters, acc_ns + "use_time_compensation_to_calc_distance",
      acc_param_->use_time_compensation_to_dist);
    update_parameter(
      parameters, acc_ns + "lowpass_gain_of_upper_velocity",
      acc_param_->lowpass_gain_);

    /* parameter for pid in acc */
    update_parameter(parameters, acc_ns + "p_coefficient_positive", acc_param_->p_coeff_pos);
    update_parameter(parameters, acc_ns + "p_coefficient_negative", acc_param_->p_coeff_neg);
    update_parameter(parameters, acc_ns + "d_coefficient_positive", acc_param_->d_coeff_pos);
    update_parameter(parameters, acc_ns + "d_coefficient_negative", acc_param_->d_coeff_neg);

    /* parameter for speed estimation of obstacle */
    update_parameter(
      parameters, acc_ns + "object_polygon_length_margin",
      acc_param_->object_polygon_length_margin);
    update_parameter(
      parameters, acc_ns + "object_polygon_width_margin",
      acc_param_->object_polygon_width_margin);
    update_parameter(
      parameters, acc_ns + "valid_estimated_vel_diff_time",
      acc_param_->valid_est_vel_diff_time);
    update_parameter(parameters, acc_ns + "valid_vel_que_time", acc_param_->valid_vel_que_time);
    update_parameter(parameters, acc_ns + "valid_estimated_vel_max", acc_param_->valid_est_vel_max);
    update_parameter(parameters, acc_ns + "valid_estimated_vel_min", acc_param_->valid_est_vel_min);
    update_parameter(parameters, acc_ns + "thresh_vel_to_stop", acc_param_->thresh_vel_to_stop);
    update_parameter(
      parameters, acc_ns + "use_rough_velocity_estimation",
      acc_param_->use_rough_est_vel);
    update_parameter(parameters, acc_ns + "rough_velocity_rate", acc_param_->rough_velocity_rate);

    planner_->updateParameters(stop_param_, slow_down_param_, acc_param_);

    result.successful = true;
    result.reason = "success";
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace obstacle_stop_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  obstacle_stop_planner::ObstacleStopPlannerNode)
