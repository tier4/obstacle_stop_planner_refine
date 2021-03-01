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
#include "obstacle_stop_planner/point_helper.hpp"
#include "obstacle_stop_planner/polygon.hpp"

#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace motion_planning
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

ObstacleStopPlannerNode::ObstacleStopPlannerNode()
: Node("obstacle_stop_planner")
, tf_buffer_(this->get_clock())
, tf_listener_(tf_buffer_)
, obstacle_pointcloud_(this->get_logger())
, vehicle_info_(vehicle_info_util::VehicleInfo::create(*this), this->get_node_parameters_interface())
{
  obstacle_pointcloud_.SetVehicleInfo(vehicle_info_);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(this, vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_);

  // Initializer
  acc_controller_ = std::make_unique<motion_planning::AdaptiveCruiseController>(
    this, vehicle_info_.vehicle_width_m_, vehicle_info_.vehicle_length_m_, vehicle_info_.wheel_base_m_, vehicle_info_.front_overhang_m_);

  // Publishers
  path_pub_ =
    this->create_publisher<autoware_planning_msgs::msg::Trajectory>("output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("output/stop_reason", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", 1,
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/twist", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  dynamic_object_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1));
  expand_stop_range_sub_ = this->create_subscription<autoware_debug_msgs::msg::Float32Stamped>(
    "input/expand_stop_range", 1,
    std::bind(
      &ObstacleStopPlannerNode::externalExpandStopRangeCallback, this, std::placeholders::_1));
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  obstacle_pointcloud_.SetPointCloud(input_msg);
}
void ObstacleStopPlannerNode::pathCallback(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg)
{
  if (!obstacle_pointcloud_.IsDataReceived()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && vehicle_info_.enable_slow_down_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  /*
   * extend trajectory to consider obstacles after the goal
   */
  autoware_planning_msgs::msg::Trajectory extended_trajectory;
  trajectory_.extendTrajectory(*input_msg, vehicle_info_.extend_distance_, extended_trajectory);

  const autoware_planning_msgs::msg::Trajectory base_path = *input_msg;
  autoware_planning_msgs::msg::Trajectory output_msg = *input_msg;
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;

  /*
   * trim trajectory from self pose
   */
  geometry_msgs::msg::Pose self_pose;
  getSelfPose(input_msg->header, tf_buffer_, self_pose);
  autoware_planning_msgs::msg::Trajectory trim_trajectory;
  size_t trajectory_trim_index;
  trajectory_.trimTrajectoryWithIndexFromSelfPose(base_path, self_pose, trim_trajectory, trajectory_trim_index);

  /*
   * decimate trajectory for calculation cost
   */
  autoware_planning_msgs::msg::Trajectory decimate_trajectory;
  std::map<size_t /* decimate */, size_t /* origin */> decimate_trajectory_index_map;
  trajectory_.decimateTrajectory(
    trim_trajectory, vehicle_info_.step_length_, vehicle_info_, decimate_trajectory, decimate_trajectory_index_map);

  autoware_planning_msgs::msg::Trajectory & trajectory = decimate_trajectory;

  /*
   * search candidate obstacle pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  // search obstacle candidate pointcloud to reduce calculation cost
  const double search_radius = vehicle_info_.getSearchRadius();
  obstacle_pointcloud_.SetSearchRadius(search_radius);

  obstacle_pointcloud_.SearchCandidateObstacle(tf_buffer_, trajectory);

  /*
   * check collision, slow_down
   */
  // for collision
  bool is_collision = false;
  size_t decimate_trajectory_collision_index;
  pcl::PointXYZ nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool candidate_slow_down = false;
  bool is_slow_down = false;
  size_t decimate_trajectory_slow_down_index;
  pcl::PointXYZ nearest_slow_down_point;
  pcl::PointXYZ lateral_nearest_slow_down_point;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  double lateral_deviation = 0.0;
  PointHelper point_helper;
  point_helper.SetVehicleInfo(vehicle_info_);

  for (int i = 0; i < static_cast<int>(trajectory.points.size()) - 1; ++i) {
    /*
     * create one step circle center for vehicle
     */
    const auto prev_center_pose = vehicle_info_.getVehicleCenterFromBase(trajectory.points.at(i).pose);
    Point prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = vehicle_info_.getVehicleCenterFromBase(trajectory.points.at(i + 1).pose);
    Point next_center_point(next_center_pose.position.x, next_center_pose.position.y);
    /*
     * create one step polygon for vehicle
     */
    std::vector<cv::Point2d> one_step_move_vehicle_polygon = createOneStepPolygon(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
      vehicle_info_.expand_stop_range_);
    debug_ptr_->pushPolygon(
      one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z, PolygonType::Vehicle);
    // convert boost polygon
    auto boost_one_step_move_vehicle_polygon = ConvertToBoostPolygon(one_step_move_vehicle_polygon);

    std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
    Polygon boost_one_step_move_slow_down_range_polygon;
    if (vehicle_info_.enable_slow_down_) {
      /*
      * create one step polygon for slow_down range
      */
      one_step_move_slow_down_range_polygon = createOneStepPolygon(
        trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
        vehicle_info_.expand_slow_down_range_);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDownRange);
      // convert boost polygon
      boost_one_step_move_slow_down_range_polygon = ConvertToBoostPolygon(one_step_move_slow_down_range_polygon);
    }

    // check within polygon
    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;
    if (!is_slow_down && vehicle_info_.enable_slow_down_) {
      for (size_t j = 0; j < obstacle_candidate_pointcloud_ptr->size(); ++j) {
        Point point(
          obstacle_candidate_pointcloud_ptr->at(j).x, obstacle_candidate_pointcloud_ptr->at(j).y);
        if (
          bg::distance(prev_center_point, point) < vehicle_info_.slow_down_search_radius_ ||
          bg::distance(next_center_point, point) < vehicle_info_.slow_down_search_radius_)
        {
          if (bg::within(point, boost_one_step_move_slow_down_range_polygon)) {
            slow_down_pointcloud_ptr->push_back(obstacle_candidate_pointcloud_ptr->at(j));
            candidate_slow_down = true;
          }
        }
      }
    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }
    for (size_t j = 0; j < slow_down_pointcloud_ptr->size(); ++j) {
      Point point(slow_down_pointcloud_ptr->at(j).x, slow_down_pointcloud_ptr->at(j).y);
      if (
        bg::distance(prev_center_point, point) < vehicle_info_.stop_search_radius_ ||
        bg::distance(next_center_point, point) < vehicle_info_.stop_search_radius_)
      {
        if (bg::within(point, boost_one_step_move_vehicle_polygon)) {
          collision_pointcloud_ptr->push_back(slow_down_pointcloud_ptr->at(j));
          is_collision = true;
          debug_ptr_->pushPolygon(
            one_step_move_vehicle_polygon, trajectory.points.at(i).pose.position.z,
            PolygonType::Collision);
        }
      }
    }
    if (candidate_slow_down && !is_collision && !is_slow_down) {
      is_slow_down = true;
      decimate_trajectory_slow_down_index = i;
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDown);
      point_helper.getNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose, &nearest_slow_down_point,
        &nearest_collision_point_time);
      point_helper.getLateralNearestPoint(
        *slow_down_pointcloud_ptr, trajectory.points.at(i).pose, &lateral_nearest_slow_down_point,
        &lateral_deviation);
      debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
    }

    /*
     * search nearest collision point by beginning of path
     */
    if (is_collision) {
      point_helper.getNearestPoint(
        *collision_pointcloud_ptr, trajectory.points.at(i).pose, &nearest_collision_point,
        &nearest_collision_point_time);
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
    acc_controller_->insertAdaptiveCruiseVelocity(
      decimate_trajectory, decimate_trajectory_collision_index, self_pose, nearest_collision_point,
      nearest_collision_point_time, object_ptr_, current_velocity_ptr_, need_to_stop, output_msg);
  }

  /*
   * insert stop point
   */
  if (need_to_stop) {
    for (size_t i = decimate_trajectory_index_map.at(decimate_trajectory_collision_index) +
      trajectory_trim_index;
      i < base_path.points.size(); ++i)
    {
      Eigen::Vector2d trajectory_vec;
      {
        const double yaw =
          getYawFromGeometryMsgsQuaternion(base_path.points.at(i).pose.orientation);
        trajectory_vec << std::cos(yaw), std::sin(yaw);
      }
      Eigen::Vector2d collision_point_vec;
      collision_point_vec << nearest_collision_point.x - base_path.points.at(i).pose.position.x,
        nearest_collision_point.y - base_path.points.at(i).pose.position.y;

      if (
        trajectory_vec.dot(collision_point_vec) < 0.0 ||
        (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec)))
      {
        const auto stop_point =
          point_helper.searchInsertPoint(i, base_path, trajectory_vec, collision_point_vec);
        if (stop_point.index <= output_msg.points.size()) {
          const auto trajectory_point = point_helper.insertStopPoint(stop_point, base_path, output_msg);
          stop_reason_diag = makeStopReasonDiag("obstacle", trajectory_point.pose);
          debug_ptr_->pushPose(trajectory_point.pose, PoseType::Stop);
        }
        break;
      }
    }
  }

  /*
   * insert slow_down point
   */
  if (is_slow_down) {
    for (size_t i = decimate_trajectory_index_map.at(decimate_trajectory_slow_down_index);
      i < base_path.points.size(); ++i)
    {
      Eigen::Vector2d trajectory_vec;
      {
        const double yaw =
          getYawFromGeometryMsgsQuaternion(base_path.points.at(i).pose.orientation);
        trajectory_vec << std::cos(yaw), std::sin(yaw);
      }
      Eigen::Vector2d slow_down_point_vec;
      slow_down_point_vec << nearest_slow_down_point.x - base_path.points.at(i).pose.position.x,
        nearest_slow_down_point.y - base_path.points.at(i).pose.position.y;

      if (
        trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
        (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec)))
      {
        const double slow_down_target_vel = calcSlowDownTargetVel(lateral_deviation);
        const auto slow_down_start_point = point_helper.createSlowDownStartPoint(
          i, vehicle_info_.slow_down_margin_, slow_down_target_vel, trajectory_vec, slow_down_point_vec,
          base_path, current_velocity_ptr_->twist.linear.x);

        if (slow_down_start_point.index <= output_msg.points.size()) {
          const auto slowdown_trajectory_point = point_helper.insertSlowDownStartPoint(slow_down_start_point, base_path, output_msg);
          debug_ptr_->pushPose(slowdown_trajectory_point.pose, PoseType::SlowDownStart);
          insertSlowDownVelocity(
            slow_down_start_point.index, slow_down_target_vel, slow_down_start_point.velocity,
            output_msg);
        }
        break;
      }
    }
  }
  path_pub_->publish(output_msg);
  stop_reason_diag_pub_->publish(stop_reason_diag);
  debug_ptr_->publish();
}

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const autoware_debug_msgs::msg::Float32Stamped::ConstSharedPtr input_msg)
{
  vehicle_info_.expand_stop_range_ = input_msg->data;
  vehicle_info_.stop_search_radius_ =
    vehicle_info_.step_length_ + std::hypot(vehicle_info_.vehicle_width_m_ / 2.0 + vehicle_info_.expand_stop_range_, vehicle_info_.vehicle_length_m_ / 2.0);
}

void ObstacleStopPlannerNode::insertSlowDownVelocity(
  const size_t slow_down_start_point_idx, const double slow_down_target_vel, double slow_down_vel,
  autoware_planning_msgs::msg::Trajectory & output_path)
{
  autoware_planning_msgs::msg::TrajectoryPoint slow_down_end_trajectory_point;
  bool is_slow_down_end = false;

  for (size_t j = slow_down_start_point_idx; j < output_path.points.size() - 1; ++j) {
    output_path.points.at(j).twist.linear.x =
      std::min(slow_down_vel, output_path.points.at(j).twist.linear.x);
    const auto dist = std::hypot(
      output_path.points.at(j).pose.position.x - output_path.points.at(j + 1).pose.position.x,
      output_path.points.at(j).pose.position.y - output_path.points.at(j + 1).pose.position.y);
    slow_down_vel = std::max(
      slow_down_target_vel,
      std::sqrt(std::max(slow_down_vel * slow_down_vel - 2 * vehicle_info_.max_deceleration_ * dist, 0.0)));
    if (!is_slow_down_end && slow_down_vel <= slow_down_target_vel) {
      slow_down_end_trajectory_point = output_path.points.at(j + 1);
      is_slow_down_end = true;
    }
  }
  if (!is_slow_down_end) {
    slow_down_end_trajectory_point = output_path.points.back();
    is_slow_down_end = true;
  }
  debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
}

double ObstacleStopPlannerNode::calcSlowDownTargetVel(const double lateral_deviation)
{
  return vehicle_info_.min_slow_down_vel_ + (vehicle_info_.max_slow_down_vel_ - vehicle_info_.min_slow_down_vel_) *
    std::max(lateral_deviation - vehicle_info_.vehicle_width_m_ / 2, 0.0) /
    vehicle_info_.expand_slow_down_range_;
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

std::vector<cv::Point2d> ObstacleStopPlannerNode::createOneStepPolygon(
  const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
  const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;
  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) -
        std::sin(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) +
        std::cos(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-vehicle_info_.rear_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-vehicle_info_.rear_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-vehicle_info_.rear_overhang_m_) -
        std::sin(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-vehicle_info_.rear_overhang_m_) +
        std::cos(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width)));
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) -
        std::sin(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) +
        std::cos(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * (vehicle_info_.wheel_base_m_ + vehicle_info_.front_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-vehicle_info_.rear_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-vehicle_info_.rear_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_.vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-vehicle_info_.rear_overhang_m_) -
        std::sin(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-vehicle_info_.rear_overhang_m_) +
        std::cos(yaw) * (vehicle_info_.vehicle_width_m_ / 2.0 + expand_width)));
  }
  return convexHull(one_step_move_vehicle_corner_points);
}

std::vector<cv::Point2d> ObstacleStopPlannerNode::convexHull(
  const std::vector<cv::Point2d> pointcloud)
{
  auto centroid = calcCentroid(pointcloud);

  std::vector<cv::Point> normalized_pointcloud;
  std::vector<cv::Point> normalized_polygon_points;
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    normalized_pointcloud.push_back(
      cv::Point(
        (pointcloud.at(i).x - centroid.x) * 1000.0, (pointcloud.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_pointcloud, normalized_polygon_points);

  std::vector<cv::Point2d> polygon_points{normalized_polygon_points.size()};
  for (size_t i = 0; i < normalized_polygon_points.size(); ++i) {
    cv::Point2d polygon_point;
    polygon_point.x = (normalized_polygon_points.at(i).x / 1000.0 + centroid.x);
    polygon_point.y = (normalized_polygon_points.at(i).y / 1000.0 + centroid.y);
    polygon_points.emplace_back(polygon_point);
  }
  return polygon_points;
}

bool ObstacleStopPlannerNode::getSelfPose(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & self_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
    transform =
      tf_buffer.lookupTransform(
      header.frame_id, "base_link", header.stamp, rclcpp::Duration::from_seconds(
        0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  } catch (tf2::TransformException & ex) {
    return false;
  }
}
}  // namespace motion_planning
