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

#include <memory>
#include <string>

#include "obstacle_stop_planner_nodes/obstacle_stop_planner_node.hpp"
#include "vehicle_info_util/vehicle_info.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"

namespace obstacle_stop_planner_nodes
{

using rclcpp::QoS;

ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node{"obstacle_stop_planner", node_options},
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  is_pointcloud_received_(false)
{
  // Vehicle Info
  VehicleInfo vehicle_info;
  auto i = vehicle_info_util::VehicleInfo::create(*this);
  vehicle_info.wheel_radius = i.wheel_radius_m_;
  vehicle_info.wheel_width = i.wheel_width_m_;
  vehicle_info.wheel_base = i.wheel_base_m_;
  vehicle_info.wheel_tread = i.wheel_tread_m_;
  vehicle_info.front_overhang = i.front_overhang_m_;
  vehicle_info.rear_overhang = i.rear_overhang_m_;
  vehicle_info.left_overhang = i.left_overhang_m_;
  vehicle_info.right_overhang = i.right_overhang_m_;
  vehicle_info.vehicle_height = i.vehicle_height_m_;
  vehicle_info.vehicle_length = i.vehicle_length_m_;
  vehicle_info.vehicle_width = i.vehicle_width_m_;
  vehicle_info.min_longitudinal_offset = i.min_longitudinal_offset_m_;
  vehicle_info.max_longitudinal_offset = i.max_longitudinal_offset_m_;
  vehicle_info.min_lateral_offset = i.min_lateral_offset_m_;
  vehicle_info.max_lateral_offset = i.max_lateral_offset_m_;
  vehicle_info.min_height_offset = i.min_height_offset_m_;
  vehicle_info.max_height_offset = i.max_height_offset_m_;

  // Parameters for stop_planner
  obstacle_stop_planner::StopControlParameter stop_param;
  stop_param.stop_margin = declare_parameter("stop_planner.stop_margin", 5.0);
  stop_param.min_behavior_stop_margin = declare_parameter(
    "stop_planner.min_behavior_stop_margin",
    2.0);
  stop_param.step_length = declare_parameter("stop_planner.step_length", 1.0);
  stop_param.extend_distance = declare_parameter("stop_planner.extend_distance", 0.0);
  stop_param.expand_stop_range = declare_parameter("stop_planner.expand_stop_range", 0.0);
  stop_param.stop_margin += vehicle_info.wheel_base + vehicle_info.front_overhang;
  stop_param.min_behavior_stop_margin +=
    vehicle_info.wheel_base + vehicle_info.front_overhang;
  stop_param.stop_search_radius = stop_param.step_length + std::hypot(
    vehicle_info.vehicle_width / 2.0 + stop_param.expand_stop_range,
    vehicle_info.vehicle_length / 2.0);

  // Parameters for slow_down_planner
  obstacle_stop_planner::SlowDownControlParameter slow_down_param;
  slow_down_param.slow_down_margin = declare_parameter("slow_down_planner.slow_down_margin", 5.0);
  slow_down_param.expand_slow_down_range =
    declare_parameter("slow_down_planner.expand_slow_down_range", 1.0);
  slow_down_param.max_slow_down_vel = declare_parameter("slow_down_planner.max_slow_down_vel", 4.0);
  slow_down_param.min_slow_down_vel = declare_parameter("slow_down_planner.min_slow_down_vel", 2.0);
  slow_down_param.max_deceleration = declare_parameter("slow_down_planner.max_deceleration", 2.0);
  slow_down_param.enable_slow_down = declare_parameter("enable_slow_down", false);
  slow_down_param.slow_down_margin += vehicle_info.wheel_base + vehicle_info.front_overhang;
  slow_down_param.slow_down_search_radius = stop_param.step_length + std::hypot(
    vehicle_info.vehicle_width / 2.0 + slow_down_param.expand_slow_down_range,
    vehicle_info.vehicle_length / 2.0);

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
    vehicle_info,
    stop_param,
    slow_down_param,
    acc_param);

  enable_slow_down_ = slow_down_param.enable_slow_down;

  // Publishers
  path_pub_ =
    this->create_publisher<Trajectory>("output/trajectory", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<PointCloud2>(
    "input/pointcloud", QoS{1},
    [this](const PointCloud2::SharedPtr msg) {this->obstaclePointcloudCallback(msg);});

  path_sub_ = this->create_subscription<Trajectory>(
    "input/trajectory", QoS{1},
    [this](const Trajectory::SharedPtr msg) {this->pathCallback(msg);});

  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", QoS{1},
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
      this->current_velocity_ptr_ = msg;
    });

  dynamic_object_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "input/objects", QoS{1},
    [this](const autoware_perception_msgs::msg::DynamicObjectArray::SharedPtr msg) {
      this->object_ptr_ = msg;
    });

  expand_stop_range_sub_ = this->create_subscription<autoware_debug_msgs::msg::Float32Stamped>(
    "input/expand_stop_range", QoS{1},
    [this](const autoware_debug_msgs::msg::Float32Stamped::SharedPtr msg) {
      this->externalExpandStopRangeCallback(msg->data);
    });
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const PointCloud2::SharedPtr input_msg)
{
  planner_->updatePointCloud(input_msg);
  pointcloud_header_ = input_msg->header;
  is_pointcloud_received_ = true;
}

void ObstacleStopPlannerNode::pathCallback(
  const Trajectory::SharedPtr input_msg)
{
  if (!is_pointcloud_received_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  auto self_pose = getSelfPose(input_msg->header, tf_buffer_);

  const auto trajectory_set = planner_->processTrajectory(*input_msg, self_pose);

  const auto transform_stamped = getTransform(
    trajectory_set.decimate.header, pointcloud_header_,
    tf_buffer_);

  const auto output_msg = planner_->updatePath(
    trajectory_set,
    self_pose,
    transform_stamped,
    current_velocity_ptr_->twist.angular.x,
    this->now());

  path_pub_->publish(output_msg);
  // debug_ptr_->publish();
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(const float expand_stop_range)
{
  planner_->updateExpandStopRange(expand_stop_range);
}

Pose ObstacleStopPlannerNode::getSelfPose(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer)
{
  Pose self_pose;
  try {
    const auto transform =
      tf_buffer.lookupTransform(
      header.frame_id, "base_link", header.stamp, rclcpp::Duration::from_seconds(
        0.1));
    self_pose = autoware_utils::transform2pose(transform.transform);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "could not get self pose from tf_buffer.");
  }
  return self_pose;
}

geometry_msgs::msg::TransformStamped
ObstacleStopPlannerNode::getTransform(
  const std_msgs::msg::Header & target,
  const std_msgs::msg::Header & source,
  const tf2_ros::Buffer & tf_buffer)
{
  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
      target.frame_id, source.frame_id,
      source.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      this->get_logger(),
      "[obstacle_stop_planner] Failed to look up transform from " <<
        target.frame_id << " to " << source.frame_id);
    // do not publish path
    // FIXME: エラーハンドリングの方法を考える
  }
  return transform_stamped;
}

}  // namespace obstacle_stop_planner_nodes

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  obstacle_stop_planner_nodes::ObstacleStopPlannerNode)
