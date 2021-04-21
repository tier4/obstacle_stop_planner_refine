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
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "obstacle_stop_planner/obstacle_stop_planner.hpp"
#include "obstacle_stop_planner/util.hpp"
#include "obstacle_stop_planner/one_step_polygon.hpp"
#include "vehicle_info_util/vehicle_info.hpp"


namespace obstacle_stop_planner
{
ObstacleStopPlanner::ObstacleStopPlanner(
  const rclcpp::node_interfaces::NodeLoggingInterface::ConstSharedPtr node_logging,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock,
  const std::shared_ptr<vehicle_info_util::VehicleInfo> & vehicle_info,
  const std::shared_ptr<StopControlParameter> & stop_param,
  const std::shared_ptr<SlowDownControlParameter> & slow_down_param,
  const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param)
: node_clock_(node_clock),
  vehicle_info_(vehicle_info)
{
  // Initializer
  acc_controller_ = std::make_unique<obstacle_stop_planner::AdaptiveCruiseController>(
    node_logging, node_clock, vehicle_info, acc_param);

  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
    node_clock,
    vehicle_info_->wheel_base_m_ +
    vehicle_info_->front_overhang_m_);

  updateParameters(stop_param, slow_down_param, acc_param);
}

void ObstacleStopPlanner::updateParameters(
  const std::shared_ptr<StopControlParameter> & stop_param,
  const std::shared_ptr<SlowDownControlParameter> & slow_down_param,
  const std::shared_ptr<AdaptiveCruiseControlParameter> & acc_param)
{
  stop_param_ = stop_param;
  slow_down_param_ = slow_down_param;
  acc_param_ = acc_param;

  {
    const auto & i = *vehicle_info_;

    stop_param_->stop_margin += i.wheel_base_m_ + i.front_overhang_m_;
    stop_param_->min_behavior_stop_margin +=
      i.wheel_base_m_ + i.front_overhang_m_;
    slow_down_param_->slow_down_margin += i.wheel_base_m_ + i.front_overhang_m_;
    if (slow_down_param_->enable_slow_down) {
      search_radius_ = stop_param->step_length + std::hypot(
        i.vehicle_width_m_ / 2.0 + slow_down_param->expand_slow_down_range,
        i.vehicle_length_m_ / 2.0);
    } else {
      search_radius_ = stop_param->step_length + std::hypot(
        i.vehicle_width_m_ / 2.0 + stop_param->expand_stop_range,
        i.vehicle_length_m_ / 2.0);
    }
  }

  acc_controller_->updateParameter(acc_param_);
}

Output ObstacleStopPlanner::processTrajectory(const Input & input)
{
  // Create debug instance
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
    node_clock_,
    vehicle_info_->wheel_base_m_ +
    vehicle_info_->front_overhang_m_);

  // Search obstacles near vehicle
  const auto obstacles = searchCandidateObstacle(input.input_trajectory, input.obstacle_pointcloud);

  // Find collision point
  const auto nearest_collision = findCollisionPoint(input.input_trajectory, obstacles);

  // Find nearest point inside slow down detection area
  const auto nearest_slow_down = findSlowDownPoint(input.input_trajectory, obstacles);

  Output output;

  // Limit trajectory for adaptive cruise control
  if (nearest_collision.has_value()) {
    const auto acc_trajectory = planAdaptiveCruise(input, nearest_collision.value());

    if (acc_trajectory.has_value()) {
      // Set adaptive cruise trajectory as output
      output.output_trajectory = acc_trajectory.value();
      return output;
    }
  }

  auto tmp_trajectory = input.input_trajectory;

  // Slow down if obstacle is in the detection area
  if (nearest_slow_down.has_value()) {
    tmp_trajectory = planSlowDown(tmp_trajectory, nearest_slow_down.value(), obstacles);
  }

  // Stop if obstacle is in the detection area
  if (nearest_collision.has_value()) {
    tmp_trajectory = planObstacleStop(tmp_trajectory, nearest_collision.value());
  }

  // Set output
  output.output_trajectory = tmp_trajectory;
  output.acc_debug_msg = acc_controller_->getDebugMsg();
  output.stop_reason = debug_ptr_->makeStopReasonArray();
  output.debug_viz_msg = debug_ptr_->makeVisualizationMarker();

  return output;
}

// search obstacle candidate pointcloud to reduce calculation cost
std::vector<Point3d> ObstacleStopPlanner::searchCandidateObstacle(
  const Trajectory & trajectory, const std::vector<Point3d> & obstacle_pointcloud)
{
  std::vector<Point3d> filtered_points;

  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = getVehicleCenterFromBase(
      trajectory_point.pose,
      vehicle_info_->vehicle_length_m_,
      vehicle_info_->rear_overhang_m_);
    const auto center_point = autoware_utils::fromMsg(center_pose.position);

    for (const auto & point : obstacle_pointcloud) {
      const auto diff = center_point - point;
      const double distance = std::hypot(diff.x(), diff.y());
      if (distance < search_radius_) {
        filtered_points.emplace_back(point);
      }
    }
  }

  return filtered_points;
}

boost::optional<Collision> ObstacleStopPlanner::findCollisionPoint(
  const Trajectory & trajectory, const std::vector<Point3d> & obstacle_points)
{
  // Create footprint vector
  const auto footprints = createStopFootprints(trajectory);
  const auto passing_areas = createVehiclePassingAreas(footprints);

  // For debugging
  std::vector<double> z;
  std::transform(
    trajectory.points.cbegin(), trajectory.points.cend(),
    std::back_inserter(z),
    [&](const auto & p) {
      return p.pose.position.z;
    });
  debug_ptr_->pushPolygons(passing_areas, z, PolygonType::Vehicle);


  // Loop for each trajectory point and areas
  //  passing_area.size() is trajectory.size() - 1
  for (size_t i = 0; i < passing_areas.size(); ++i) {
    // Check whether obstacle is inside passing area
    const auto base_point = autoware_utils::fromMsg(
      autoware_utils::getPoint(trajectory.points.at(i)));

    const auto collision_particle = findCollisionParticle(
      passing_areas.at(i),
      obstacle_points,
      base_point.to_2d());
    if (collision_particle.has_value()) {
      Collision collision;
      collision.segment_index = i;
      collision.obstacle_point = collision_particle.value().to_2d();

      // For debugging
      debug_ptr_->pushPolygon(
        passing_areas.at(i), trajectory.points.at(i).pose.position.z,
        PolygonType::Collision);
      debug_ptr_->pushObstaclePoint(collision_particle.value(), PointType::Stop);
      debug_ptr_->pushPose(trajectory.points.at(i).pose, PoseType::Stop);

      return collision;
    }
  }
  return {};
}

boost::optional<Collision> ObstacleStopPlanner::findSlowDownPoint(
  const Trajectory & trajectory, const std::vector<Point3d> & obstacle_points)
{
  // Create footprint vector
  const auto footprints = createSlowDownFootprints(trajectory);
  const auto passing_areas = createVehiclePassingAreas(footprints);

  // For debugging
  std::vector<double> z;
  std::transform(
    trajectory.points.cbegin(), trajectory.points.cend(),
    std::back_inserter(z),
    [&](const auto & p) {
      return p.pose.position.z;
    });
  debug_ptr_->pushPolygons(passing_areas, z, PolygonType::SlowDownRange);

  // Loop for each trajectory point and areas
  //  passing_area.size() is trajectory.size() - 1
  for (size_t i = 0; i < passing_areas.size(); ++i) {
    // Check whether obstacle is inside passing area
    const auto base_point = autoware_utils::fromMsg(
      autoware_utils::getPoint(trajectory.points.at(i)));

    const auto collision_particle = findCollisionParticle(
      passing_areas.at(i),
      obstacle_points,
      base_point.to_2d());
    if (collision_particle.has_value()) {
      Collision collision;
      collision.segment_index = i;
      collision.obstacle_point = collision_particle.value().to_2d();

      // For debugging
      debug_ptr_->pushPolygon(
        passing_areas.at(i), trajectory.points.at(i).pose.position.z,
        PolygonType::SlowDown);
      debug_ptr_->pushObstaclePoint(collision_particle.value(), PointType::SlowDown);
      debug_ptr_->pushPose(trajectory.points.at(i).pose, PoseType::SlowDownStart);

      return collision;
    }
  }
  return {};
}

std::vector<LinearRing2d> ObstacleStopPlanner::createStopFootprints(const Trajectory & trajectory)
{
  return createVehicleFootprints(trajectory, stop_param_->expand_stop_range);
}

std::vector<LinearRing2d> ObstacleStopPlanner::createSlowDownFootprints(
  const Trajectory & trajectory)
{
  return createVehicleFootprints(trajectory, slow_down_param_->expand_slow_down_range);
}

std::vector<LinearRing2d> ObstacleStopPlanner::createVehicleFootprints(
  const Trajectory & trajectory, const double margin)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint =
    createVehicleFootprint(*vehicle_info_, 0.0, margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.emplace_back(
      transformVector(local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<Polygon2d> ObstacleStopPlanner::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<Polygon2d> areas;
  areas.reserve(vehicle_footprints.size() - 1);
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.emplace_back(createHullFromFootprints(footprint1, footprint2));
  }

  return areas;
}

Polygon2d ObstacleStopPlanner::createHullFromFootprints(
  const LinearRing2d & area1, const LinearRing2d & area2)
{
  autoware_utils::MultiPoint2d combined;
  combined.reserve(area1.size() + area2.size());
  for (const auto & p : area1) {
    combined.emplace_back(p);
  }
  for (const auto & p : area2) {
    combined.emplace_back(p);
  }

  Polygon2d hull;
  boost::geometry::convex_hull(combined, hull);

  return hull;
}

boost::optional<Point3d> ObstacleStopPlanner::findCollisionParticle(
  const Polygon2d & area,
  const std::vector<Point3d> & obstacle_points, const Point2d & base_point)
{
  // Search all obstacle inside area
  std::vector<Point3d> collision_points;
  for (const auto & point : obstacle_points) {
    if (boost::geometry::within(point.to_2d(), area)) {
      collision_points.emplace_back(point);
    }
  }
  if (collision_points.empty()) {
    return {};
  }

  // Get nearest point
  std::vector<double> distances;
  distances.reserve(collision_points.size());
  std::transform(
    collision_points.cbegin(), collision_points.cend(),
    std::back_inserter(distances),
    [&](const Point3d & p) {
      return boost::geometry::distance(p.to_2d(), base_point);
    });

  const auto min_itr = std::min_element(distances.cbegin(), distances.cend());
  const auto min_idx = static_cast<size_t>(std::distance(distances.cbegin(), min_itr));

  return collision_points.at(min_idx);
}

/**
* @fn
* @brief Create adaptive cruise trajectory
* Follow front vehicle adaptively
* @param input trajectory
* @param collision collision point and index
* @return Trajectory
*/
boost::optional<Trajectory> ObstacleStopPlanner::planAdaptiveCruise(
  const Input & input,
  const Collision & collision)
{
  adaptive_cruise_controller::Input acc_input {
    input.input_trajectory,
    collision.segment_index,
    input.current_pose,
    collision.obstacle_point,
    input.pointcloud_header_time,
    input.object_array,
    input.current_velocity,
  };

  return acc_controller_->insertAdaptiveCruiseVelocity(acc_input);
}

/**
* @fn
* @brief Create slow down trajectory
* All velocity in trajectory set to minimum velcity after slow_down_index.
* @param input trajectory
* @param collision collision point and index
* @param obstacles associated points from pointcloud
* @return Trajectory
*/
Trajectory ObstacleStopPlanner::planSlowDown(
  const Trajectory & trajectory,
  const Collision & collision,
  const std::vector<Point3d> & obstacles)
{
  // get lateral deviation
  const auto lateral_deviation =
    autoware_utils::calcLateralDeviation(
    trajectory.points.at(
      collision.segment_index).pose, autoware_utils::toMsg(collision.obstacle_point.to_3d()));

  const auto target_velocity = calcSlowDownTargetVel(lateral_deviation);

  // loop trajectory point from segment_index to end
  Trajectory limited_trajectory = trajectory;
  for (auto && itr = limited_trajectory.points.begin() + collision.segment_index;
    itr != limited_trajectory.points.end(); ++itr)
  {
    itr->twist.linear.x = target_velocity;

    // if obstacle is not in front of point, resume speed
    if (!findFrontObstacles(*itr, obstacles)) {
      // For debugging
      debug_ptr_->pushPose(itr->pose, PoseType::SlowDownEnd);
      break;
    }
  }
  return limited_trajectory;
}

// Return true if obstacle is in front side of point
bool ObstacleStopPlanner::findFrontObstacles(
  const autoware_planning_msgs::msg::TrajectoryPoint & point,
  const std::vector<Point3d> & obstacles)
{
  for (const auto obstacle : obstacles) {
    const auto p = autoware_utils::fromMsg(point.pose.position);
    const auto diff_vec = obstacle - p;
    const auto inner_product = diff_vec.dot(p);

    // If object is in front of point
    // This means inner product value is positive
    if (inner_product > 0) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Create Stop trajectory
 * All velocity in trajectory set to 0 after stop_index.
 * @param trajectory input trajectory
 * @param collision collision point and index
 * @return Trajectory
 */
Trajectory ObstacleStopPlanner::planObstacleStop(
  const Trajectory & trajectory,
  const Collision & collision)
{
  // TODO(K.Shima): This is very simple logic. The output should be the same as the existing logic.
  Trajectory limited_trajectory = trajectory;
  for (size_t i = collision.segment_index; i < limited_trajectory.points.size(); ++i) {
    limited_trajectory.points.at(i).twist.linear.x = 0.0;
  }

  return limited_trajectory;
}

double ObstacleStopPlanner::calcSlowDownTargetVel(const double lateral_deviation) const
{
  return slow_down_param_->min_slow_down_vel +
         (slow_down_param_->max_slow_down_vel - slow_down_param_->min_slow_down_vel) *
         std::max(lateral_deviation - vehicle_info_->vehicle_width_m_ / 2, 0.0) /
         slow_down_param_->expand_slow_down_range;
}

}  // namespace obstacle_stop_planner
