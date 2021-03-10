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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <tuple>

#include "autoware_utils/geometry/geometry.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pcl/filters/voxel_grid.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "obstacle_stop_planner/node.hpp"
#include "obstacle_stop_planner/util.hpp"
#include "obstacle_stop_planner/one_step_polygon.hpp"
#include "obstacle_stop_planner/trajectory.hpp"
#include "vehicle_info_util/vehicle_info.hpp"

#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace obstacle_stop_planner
{
ObstacleStopPlannerNode::ObstacleStopPlannerNode()
: Node("obstacle_stop_planner"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  obstacle_pointcloud_(this->get_logger())
{
  // Vehicle Info
  auto i = vehicle_info_util::VehicleInfo::create(*this);
  param_.vehicle_info.wheel_radius = i.wheel_radius_m_;
  param_.vehicle_info.wheel_width = i.wheel_width_m_;
  param_.vehicle_info.wheel_base = i.wheel_base_m_;
  param_.vehicle_info.wheel_tread = i.wheel_tread_m_;
  param_.vehicle_info.front_overhang = i.front_overhang_m_;
  param_.vehicle_info.rear_overhang = i.rear_overhang_m_;
  param_.vehicle_info.left_overhang = i.left_overhang_m_;
  param_.vehicle_info.right_overhang = i.right_overhang_m_;
  param_.vehicle_info.vehicle_height = i.vehicle_height_m_;
  param_.vehicle_info.vehicle_length = i.vehicle_length_m_;
  param_.vehicle_info.vehicle_width = i.vehicle_width_m_;
  param_.vehicle_info.min_longitudinal_offset = i.min_longitudinal_offset_m_;
  param_.vehicle_info.max_longitudinal_offset = i.max_longitudinal_offset_m_;
  param_.vehicle_info.min_lateral_offset = i.min_lateral_offset_m_;
  param_.vehicle_info.max_lateral_offset = i.max_lateral_offset_m_;
  param_.vehicle_info.min_height_offset = i.min_height_offset_m_;
  param_.vehicle_info.max_height_offset = i.max_height_offset_m_;

  // Parameters
  param_.stop_margin = declare_parameter("stop_planner.stop_margin", 5.0);
  param_.min_behavior_stop_margin = declare_parameter("stop_planner.min_behavior_stop_margin", 2.0);
  param_.step_length = declare_parameter("stop_planner.step_length", 1.0);
  param_.extend_distance = declare_parameter("stop_planner.extend_distance", 0.0);
  param_.expand_stop_range = declare_parameter("stop_planner.expand_stop_range", 0.0);

  param_.slow_down_margin = declare_parameter("slow_down_planner.slow_down_margin", 5.0);
  param_.expand_slow_down_range =
    declare_parameter("slow_down_planner.expand_slow_down_range", 1.0);
  param_.max_slow_down_vel = declare_parameter("slow_down_planner.max_slow_down_vel", 4.0);
  param_.min_slow_down_vel = declare_parameter("slow_down_planner.min_slow_down_vel", 2.0);
  param_.max_deceleration = declare_parameter("slow_down_planner.max_deceleration", 2.0);
  param_.enable_slow_down = declare_parameter("enable_slow_down", false);

  param_.stop_margin += param_.vehicle_info.wheel_base + param_.vehicle_info.front_overhang;
  param_.min_behavior_stop_margin +=
    param_.vehicle_info.wheel_base + param_.vehicle_info.front_overhang;
  param_.slow_down_margin += param_.vehicle_info.wheel_base + param_.vehicle_info.front_overhang;
  param_.stop_search_radius = param_.step_length + std::hypot(
    param_.vehicle_info.vehicle_width / 2.0 + param_.expand_stop_range,
    param_.vehicle_info.vehicle_length / 2.0);
  param_.slow_down_search_radius = param_.step_length + std::hypot(
    param_.vehicle_info.vehicle_width / 2.0 + param_.expand_slow_down_range,
    param_.vehicle_info.vehicle_length / 2.0);

  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
    this,
    param_.vehicle_info.wheel_base +
    param_.vehicle_info.front_overhang);

  // Initializer
  acc_controller_ = std::make_unique<obstacle_stop_planner::AdaptiveCruiseController>(
    this, param_.vehicle_info.vehicle_width, param_.vehicle_info.vehicle_length,
    param_.vehicle_info.wheel_base, param_.vehicle_info.front_overhang);

  // Publishers
  path_pub_ =
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);

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
  obstacle_pointcloud_.updatePointCloud(input_msg);
}
void ObstacleStopPlannerNode::pathCallback(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  if (!obstacle_pointcloud_.isDataReceived()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && param_.enable_slow_down) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  /*
   * extend trajectory to consider obstacles after the goal
   */
  autoware_planning_msgs::msg::Trajectory extended_trajectory;
  trajectory_.extendTrajectory(*input_msg, param_, extended_trajectory);

  const autoware_planning_msgs::msg::Trajectory base_path = extended_trajectory;
  autoware_planning_msgs::msg::Trajectory output_msg = *input_msg;

  /*
   * trim trajectory from self pose
   */
  auto self_pose = getSelfPose(input_msg->header, tf_buffer_);
  autoware_planning_msgs::msg::Trajectory trim_trajectory;
  size_t trajectory_trim_index;
  std::tie(trim_trajectory, trajectory_trim_index) =
    trimTrajectoryWithIndexFromSelfPose(base_path, self_pose);

  /*
   * decimate trajectory for calculation cost
   */
  DecimateTrajectoryMap decimate_trajectory_map = decimateTrajectory(
    trim_trajectory, param_.step_length, param_);

  autoware_planning_msgs::msg::Trajectory & trajectory =
    decimate_trajectory_map.decimate_trajectory;

  /*
   * search candidate obstacle pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  // search obstacle candidate pointcloud to reduce calculation cost
  obstacle_pointcloud_.searchCandidateObstacle(tf_buffer_, trajectory, param_);

  /*
   * check collision, slow_down
   */
  // for collision
  bool is_collision = false;
  size_t decimate_trajectory_collision_index;
  Point3d nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool candidate_slow_down = false;
  bool is_slow_down = false;
  size_t decimate_trajectory_slow_down_index;
  Point3d nearest_slow_down_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double lateral_deviation = 0.0;
  PointHelper point_helper {param_};

  for (int i = 0; i < static_cast<int>(trajectory.points.size()) - 1; ++i) {
    /*
     * create one step circle center for vehicle
     */
    const auto prev_center_pose = getVehicleCenterFromBase(
      trajectory.points.at(i).pose,
      param_.vehicle_info.vehicle_length,
      param_.vehicle_info.rear_overhang);
    const auto next_center_pose = getVehicleCenterFromBase(
      trajectory.points.at(i + 1).pose,
      param_.vehicle_info.vehicle_length,
      param_.vehicle_info.rear_overhang);

    Point2d prev_center_point = autoware_utils::fromMsg(prev_center_pose.position).to_2d();
    Point2d next_center_point = autoware_utils::fromMsg(next_center_pose.position).to_2d();

    /*
     * create one step polygon for vehicle
     */
    const auto move_vehicle_polygon = createOneStepPolygon(
      trajectory.points.at(i).pose,
      trajectory.points.at(i + 1).pose,
      param_.expand_stop_range,
      param_.vehicle_info);
    debug_ptr_->pushPolygon(
      move_vehicle_polygon,
      trajectory.points.at(i).pose.position.z,
      PolygonType::Vehicle);

    Polygon2d move_slow_down_range_polygon;
    if (param_.enable_slow_down) {
      /*
      * create one step polygon for slow_down range
      */
      move_slow_down_range_polygon = createOneStepPolygon(
        trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
        param_.expand_slow_down_range, param_.vehicle_info);
      debug_ptr_->pushPolygon(
        move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDownRange);
    }

    // check within polygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

    std::tie(candidate_slow_down, slow_down_pointcloud_ptr) = getSlowDownPointcloud(
      is_slow_down, param_.enable_slow_down,
      obstacle_candidate_pointcloud_ptr, prev_center_point, next_center_point,
      param_.slow_down_search_radius,
      move_slow_down_range_polygon, slow_down_pointcloud_ptr, candidate_slow_down);

    std::tie(is_collision, collision_pointcloud_ptr) = getCollisionPointcloud(
      slow_down_pointcloud_ptr, prev_center_point, next_center_point,
      param_.stop_search_radius, move_vehicle_polygon,
      trajectory.points.at(i), collision_pointcloud_ptr, is_collision);

    if (candidate_slow_down && !is_collision && !is_slow_down) {
      is_slow_down = true;
      decimate_trajectory_slow_down_index = i;
      debug_ptr_->pushPolygon(
        move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDown);

      const auto nearest_slow_down_pointstamped = point_helper.getNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
      nearest_slow_down_point = nearest_slow_down_pointstamped.point;
      nearest_collision_point_time = nearest_slow_down_pointstamped.time;

      const auto lateral_nearest_slow_down_point = point_helper.getLateralNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
      lateral_deviation = lateral_nearest_slow_down_point.deviation;
      debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
    }

    /*
     * search nearest collision point by beginning of path
     */
    if (is_collision) {
      const auto nearest_collision_pointstamped = point_helper.getNearestPoint(
        *collision_pointcloud_ptr, trajectory.points.at(i).pose);
      nearest_collision_point = nearest_collision_pointstamped.point;
      nearest_collision_point_time = nearest_collision_pointstamped.time;
      debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
      decimate_trajectory_collision_index = i;
      break;
    }
  }

  /*
   * insert max velocity and judge if there is a need to stop
   */
  bool need_to_stop = is_collision;
  if (is_collision) {
    std::tie(need_to_stop, output_msg) = acc_controller_->insertAdaptiveCruiseVelocity(
      decimate_trajectory_map.decimate_trajectory,
      decimate_trajectory_collision_index,
      self_pose, nearest_collision_point.to_2d(),
      nearest_collision_point_time, object_ptr_,
      current_velocity_ptr_,
      output_msg);
  }

  /*
   * insert stop point
   */
  if (need_to_stop) {
    output_msg = insertStopPoint(
      decimate_trajectory_map.index_map.at(decimate_trajectory_collision_index) +
      trajectory_trim_index,
      base_path,
      nearest_collision_point.to_2d(),
      output_msg);
  }

  /*
   * insert slow_down point
   */
  if (is_slow_down) {
    output_msg = insertSlowDownPoint(
      decimate_trajectory_map.index_map.at(decimate_trajectory_slow_down_index),
      base_path,
      nearest_slow_down_point.to_2d(),
      calcSlowDownTargetVel(lateral_deviation),
      param_.slow_down_margin,
      output_msg);
  }
  path_pub_->publish(output_msg);
  debug_ptr_->publish();
}

// collision
std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr>
ObstacleStopPlannerNode::getCollisionPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
  const Point2d & prev_center_point,
  const Point2d & next_center_point,
  const double search_radius,
  const Polygon2d & one_step_polygon,
  const autoware_planning_msgs::msg::TrajectoryPoint & trajectory_point,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud,
  const bool is_collision)
{
  auto output_pointcloud = collision_pointcloud;
  auto output_collision = is_collision;

  for (size_t j = 0; j < slow_down_pointcloud->size(); ++j) {
    Point2d point(slow_down_pointcloud->at(j).x, slow_down_pointcloud->at(j).y);
    if (
      boost::geometry::distance(prev_center_point, point) < search_radius ||
      boost::geometry::distance(next_center_point, point) < search_radius)
    {
      if (boost::geometry::within(point, one_step_polygon)) {
        output_pointcloud->push_back(slow_down_pointcloud->at(j));
        output_collision = true;
        debug_ptr_->pushPolygon(
          one_step_polygon, trajectory_point.pose.position.z,
          PolygonType::Collision);
      }
    }
  }
  return std::make_tuple(output_collision, output_pointcloud);
}

// slow down
std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr>
ObstacleStopPlannerNode::getSlowDownPointcloud(
  const bool is_slow_down,
  const bool enable_slow_down,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud,
  const Point2d & prev_center_point,
  const Point2d & next_center_point,
  const double search_radius,
  const Polygon2d & one_step_polygon,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
  const bool candidate_slow_down)
{
  auto output_pointcloud = slow_down_pointcloud;
  auto output_candidate = candidate_slow_down;

  if (!is_slow_down && enable_slow_down) {
    for (size_t j = 0; j < obstacle_candidate_pointcloud->size(); ++j) {
      Point2d point(
        obstacle_candidate_pointcloud->at(j).x, obstacle_candidate_pointcloud->at(j).y);
      if (
        boost::geometry::distance(prev_center_point, point) < search_radius ||
        boost::geometry::distance(next_center_point, point) < search_radius)
      {
        if (boost::geometry::within(point, one_step_polygon)) {
          output_pointcloud->push_back(obstacle_candidate_pointcloud->at(j));
          output_candidate = true;
        }
      }
    }
  } else {
    output_pointcloud = obstacle_candidate_pointcloud;
  }
  return std::make_tuple(output_candidate, output_pointcloud);
}

autoware_planning_msgs::msg::Trajectory ObstacleStopPlannerNode::insertSlowDownPoint(
  const size_t search_start_index,
  const autoware_planning_msgs::msg::Trajectory & base_path,
  const Point2d & nearest_slow_down_point,
  const double slow_down_target_vel, const double slow_down_margin,
  const autoware_planning_msgs::msg::Trajectory & input_msg)
{
  auto output_msg = input_msg;
  PointHelper point_helper {param_};

  for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
    const double yaw =
      getYawFromQuaternion(base_path.points.at(i).pose.orientation);
    const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
    const Point2d slow_down_point_vec {
      nearest_slow_down_point.x() - base_path.points.at(i).pose.position.x,
      nearest_slow_down_point.y() - base_path.points.at(i).pose.position.y};

    if (
      trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
      (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec)))
    {
      const auto slow_down_start_point = point_helper.createSlowDownStartPoint(
        i, slow_down_margin, slow_down_target_vel, trajectory_vec, slow_down_point_vec,
        base_path, current_velocity_ptr_->twist.linear.x);

      if (slow_down_start_point.index <= output_msg.points.size()) {
        autoware_planning_msgs::msg::TrajectoryPoint slowdown_trajectory_point;
        std::tie(slowdown_trajectory_point, output_msg) =
          point_helper.insertSlowDownStartPoint(
          slow_down_start_point, base_path, output_msg);
        debug_ptr_->pushPose(slowdown_trajectory_point.pose, PoseType::SlowDownStart);
        output_msg = insertSlowDownVelocity(
          slow_down_start_point.index, slow_down_target_vel, slow_down_start_point.velocity,
          output_msg);
      }
      break;
    }
  }
  return output_msg;
}

// stop
autoware_planning_msgs::msg::Trajectory ObstacleStopPlannerNode::insertStopPoint(
  const size_t search_start_index,
  const autoware_planning_msgs::msg::Trajectory & base_path,
  const Point2d & nearest_collision_point,
  const autoware_planning_msgs::msg::Trajectory & input_msg)
{
  auto output_msg = input_msg;
  PointHelper point_helper {param_};

  for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
    const double yaw =
      getYawFromQuaternion(base_path.points.at(i).pose.orientation);
    const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
    const Point2d collision_point_vec {
      nearest_collision_point.x() - base_path.points.at(i).pose.position.x,
      nearest_collision_point.y() - base_path.points.at(i).pose.position.y};

    if (
      trajectory_vec.dot(collision_point_vec) < 0.0 ||
      (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec)))
    {
      const auto stop_point =
        point_helper.searchInsertPoint(i, base_path, trajectory_vec, collision_point_vec);
      if (stop_point.index <= output_msg.points.size()) {
        autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
        std::tie(trajectory_point, output_msg) =
          point_helper.insertStopPoint(stop_point, base_path, output_msg);
        debug_ptr_->pushPose(trajectory_point.pose, PoseType::Stop);
      }
      break;
    }
  }
  return output_msg;
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const autoware_planning_msgs::msg::ExpandStopRange::ConstSharedPtr input_msg)
{
  param_.expand_stop_range = input_msg->expand_stop_range;
  param_.stop_search_radius =
    param_.step_length + std::hypot(
    param_.vehicle_info.vehicle_width / 2.0 + param_.expand_stop_range,
    param_.vehicle_info.vehicle_length / 2.0);
}

autoware_planning_msgs::msg::Trajectory ObstacleStopPlannerNode::insertSlowDownVelocity(
  const size_t slow_down_start_point_idx, const double slow_down_target_vel, double slow_down_vel,
  const autoware_planning_msgs::msg::Trajectory & input_path)
{
  autoware_planning_msgs::msg::TrajectoryPoint slow_down_end_trajectory_point;
  auto output_path = input_path;
  bool is_slow_down_end = false;

  for (size_t j = slow_down_start_point_idx; j < output_path.points.size() - 1; ++j) {
    output_path.points.at(j).twist.linear.x =
      std::min(slow_down_vel, output_path.points.at(j).twist.linear.x);
    const auto dist = std::hypot(
      output_path.points.at(j).pose.position.x - output_path.points.at(j + 1).pose.position.x,
      output_path.points.at(j).pose.position.y - output_path.points.at(j + 1).pose.position.y);
    slow_down_vel = std::max(
      slow_down_target_vel,
      std::sqrt(
        std::max(
          slow_down_vel * slow_down_vel - 2 * param_.max_deceleration * dist,
          0.0)));
    if (!is_slow_down_end && slow_down_vel <= slow_down_target_vel) {
      slow_down_end_trajectory_point = output_path.points.at(j + 1);
      is_slow_down_end = true;
    }
  }
  if (!is_slow_down_end) {
    slow_down_end_trajectory_point = output_path.points.back();
  }
  debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
  return output_path;
}

double ObstacleStopPlannerNode::calcSlowDownTargetVel(const double lateral_deviation)
{
  return param_.min_slow_down_vel +
         (param_.max_slow_down_vel - param_.min_slow_down_vel) *
         std::max(lateral_deviation - param_.vehicle_info.vehicle_width / 2, 0.0) /
         param_.expand_slow_down_range;
}

void ObstacleStopPlannerNode::dynamicObjectCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::currentVelocityCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;
}

geometry_msgs::msg::Pose ObstacleStopPlannerNode::getSelfPose(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::Pose self_pose;
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform =
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
}  // namespace obstacle_stop_planner
