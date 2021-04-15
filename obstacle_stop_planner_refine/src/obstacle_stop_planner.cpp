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
#include "obstacle_stop_planner/trajectory.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"


namespace obstacle_stop_planner
{
ObstacleStopPlanner::ObstacleStopPlanner(
  rclcpp::Node * node,
  const vehicle_info_util::VehicleInfo & vehicle_info,
  const StopControlParameter & stop_param,
  const SlowDownControlParameter & slow_down_param,
  const AdaptiveCruiseControlParameter & acc_param)
: node_(node),
  vehicle_info_(vehicle_info)
{
  updateParameters(stop_param, slow_down_param, acc_param);

  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
    node_,
    vehicle_info_.wheel_base_m_ +
    vehicle_info_.front_overhang_m_);

  // Initializer
  acc_controller_ = std::make_unique<obstacle_stop_planner::AdaptiveCruiseController>(
    node_, vehicle_info_, acc_param_);
}

void ObstacleStopPlanner::updateParameters(
  const StopControlParameter & stop_param,
  const SlowDownControlParameter & slow_down_param,
  const AdaptiveCruiseControlParameter & acc_param)
{
  stop_param_ = stop_param;
  slow_down_param_ = slow_down_param;
  acc_param_ = acc_param;

  {
    const auto & i = vehicle_info_;

    stop_param_.stop_margin += i.wheel_base_m_ + i.front_overhang_m_;
    stop_param_.min_behavior_stop_margin +=
      i.wheel_base_m_ + i.front_overhang_m_;
    slow_down_param_.slow_down_margin += i.wheel_base_m_ + i.front_overhang_m_;
    stop_param_.stop_search_radius = stop_param.step_length + std::hypot(
      i.vehicle_width_m_ / 2.0 + stop_param.expand_stop_range,
      i.vehicle_length_m_ / 2.0);
    slow_down_param_.slow_down_search_radius = stop_param.step_length + std::hypot(
      i.vehicle_width_m_ / 2.0 + slow_down_param.expand_slow_down_range,
      i.vehicle_length_m_ / 2.0);
  }
}

Output ObstacleStopPlanner::processTrajectory(const Input & input)
{
  // Search obstacles near vehicle
  const auto obstacles = searchCandidateObstacle(input.input_trajectory, input.obstacle_pointcloud);

  // Find collision point
  const auto nearest_collision = findCollisionPoint(input.input_trajectory, obstacles);

  Output output;

  // Limit trajectory for adaptive cruise control
  if (nearest_collision.has_value()) {
    const auto acc_trajectory = adaptiveCruise(input, nearest_collision.value());

    if (acc_trajectory.has_value()) {
      // Set adaptive cruise trajectory as output
      output.output_trajectory = acc_trajectory.value();
    } else {
      // Slow down if obstacle is in the detection area
      const auto slowdown_trajectory = slowDown(input.input_trajectory, nearest_collision.value());
      // Stop if obstacle is in the detection area
      output.output_trajectory = obstacleStop(slowdown_trajectory, nearest_collision.value());
    }
  } else {
    // Set input trajectory as output
    output.output_trajectory = input.input_trajectory;
  }

  return output;
}

// search obstacle candidate pointcloud to reduce calculation cost
std::vector<Point3d> ObstacleStopPlanner::searchCandidateObstacle(
  const Trajectory & trajectory, const std::vector<Point3d> & obstacle_pointcloud)
{
  std::vector<Point3d> filtered_points;
  const auto search_radius = slow_down_param_.enable_slow_down ?
    slow_down_param_.slow_down_search_radius : stop_param_.stop_search_radius;

  for (const auto & trajectory_point : trajectory.points) {
    const auto center_pose = getVehicleCenterFromBase(
      trajectory_point.pose,
      vehicle_info_.vehicle_length_m_,
      vehicle_info_.rear_overhang_m_);
    const auto center_point = autoware_utils::fromMsg(center_pose.position);

    for (const auto & point : obstacle_pointcloud) {
      const auto diff = center_point - point;
      const double distance = std::hypot(diff.x(), diff.y());
      if (distance < search_radius) {
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
  const auto footprints = createVehicleFootprints(trajectory);
  const auto passing_areas = createVehiclePassingAreas(footprints);

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
      collision.trajectory_index = i;
      collision.obstacle_point = collision_particle.value().to_2d();
      return collision;
    }
  }
  return boost::none;
}

std::vector<LinearRing2d> ObstacleStopPlanner::createVehicleFootprints(const Trajectory & trajectory)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint =
    createVehicleFootprint(vehicle_info_, 0.0, stop_param_.expand_stop_range);

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

boost::optional<Point3d> ObstacleStopPlanner::findCollisionParticle(const Polygon2d & area, const std::vector<Point3d> & obstacle_points, const Point2d & base_point)
{
  // Search all obstacle inside area
  std::vector<Point3d> collision_points;
  for (const auto & point : obstacle_points) {
    if (boost::geometry::within(point.to_2d(), area)) {
      collision_points.emplace_back(point);
    }
  }
  if (collision_points.empty()) {
    return boost::none;
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

// Create adaptive cruise trajectory
//   Follow front vehicle adaptively
// Input
//   input trajectory
//   collision
//   object array
//   vehicle's current velocity
// Output
//   output trajectory
boost::optional<Trajectory> ObstacleStopPlanner::adaptiveCruise(const Input & input, const Collision & collision)
{
  return acc_controller_->insertAdaptiveCruiseVelocity(
    input.input_trajectory,
    collision.trajectory_index,
    input.current_pose,
    collision.obstacle_point,
    input.pointcloud_header_time,
    input.object_array,
    input.current_velocity);
}

// Create Stop trajectory
//   All velocity in trajectory set to minimum velcity after slow_down_index.
// Input
//   input trajectory
//   slow_down_index
// Output
//   output trajectory
Trajectory ObstacleStopPlanner::slowDown(const Trajectory & trajectory, const Collision & collision)
{
  // get lateral deviation
  // TODO: use candidate_pointcloud instead of collision point
  const auto lateral_deviation = autoware_utils::calcLateralDeviation(trajectory.points.at(collision.trajectory_index).pose, autoware_utils::toMsg(collision.obstacle_point.to_3d()));

  const auto target_velocity = calcSlowDownTargetVel(lateral_deviation);

  // TODO: This is very simple logic. The output should be the same as the existing logic.
  Trajectory limited_trajectory = trajectory;
  for (size_t i = collision.trajectory_index; i < limited_trajectory.points.size(); ++i) {
    limited_trajectory.points.at(i).twist.linear.x = target_velocity;
  }

  return limited_trajectory;
}

// Create Stop trajectory
//   All velocity in trajectory set to 0 after stop_index.
// Input
//   input trajectory
//   stop_index
// Output
//   output trajectory
Trajectory ObstacleStopPlanner::obstacleStop(const Trajectory & trajectory, const Collision & collision)
{
  // TODO: This is very simple logic. The output should be the same as the existing logic.
  Trajectory limited_trajectory = trajectory;
  for (size_t i = collision.trajectory_index; i < limited_trajectory.points.size(); ++i) {
    limited_trajectory.points.at(i).twist.linear.x = 0.0;
  }

  return limited_trajectory;
}

////////////////////////////////////////////////////////////////
// MEMO:
// The following is a past implementation.
// I plan to remove it when I replace all the features.
////////////////////////////////////////////////////////////////

// autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::pathCallback(
//   const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr input_msg,
//   const geometry_msgs::msg::Pose & self_pose)
// {
//   /*
//    * extend trajectory to consider obstacles after the goal
//    */
//   const auto extended_trajectory = extendTrajectory(*input_msg, stop_param_.extend_distance);

//   const autoware_planning_msgs::msg::Trajectory base_path = extended_trajectory;
//   autoware_planning_msgs::msg::Trajectory output_msg = *input_msg;

//   /*
//    * trim trajectory from self pose
//    */
//   autoware_planning_msgs::msg::Trajectory trim_trajectory;
//   size_t trajectory_trim_index = 0;
//   std::tie(trim_trajectory, trajectory_trim_index) =
//     trimTrajectoryWithIndexFromSelfPose(base_path, self_pose);

//   /*
//    * decimate trajectory for calculation cost
//    */
//   DecimateTrajectoryMap decimate_trajectory_map = decimateTrajectory(
//     trim_trajectory, stop_param_.step_length);

//   autoware_planning_msgs::msg::Trajectory & trajectory =
//     decimate_trajectory_map.decimate_trajectory;

//   /*
//    * search candidate obstacle pointcloud
//    */
//   pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
//     new pcl::PointCloud<pcl::PointXYZ>);

//   // search obstacle candidate pointcloud to reduce calculation cost
//   const auto search_radius = slow_down_param_.enable_slow_down ?
//     slow_down_param_.slow_down_search_radius : stop_param_.stop_search_radius;

//   obstacle_candidate_pointcloud_ptr = searchCandidateObstacle(
//     obstacle_pointcloud_ptr_,
//     tf_buffer,
//     trajectory,
//     search_radius,
//     vehicle_info_,
//     node_->get_logger());

//   /*
//    * check collision, slow_down
//    */
//   // for collision
//   bool is_collision = false;
//   size_t decimate_trajectory_collision_index = 0;
//   Point3d nearest_collision_point;
//   rclcpp::Time nearest_collision_point_time;
//   // for slow down
//   bool candidate_slow_down = false;
//   bool is_slow_down = false;
//   size_t decimate_trajectory_slow_down_index = 0;
//   Point3d nearest_slow_down_point;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//   double lateral_deviation = 0.0;

//   for (int i = 0; i < static_cast<int>(trajectory.points.size()) - 1; ++i) {
//     /*
//      * create one step circle center for vehicle
//      */
//     const auto prev_center_pose = getVehicleCenterFromBase(
//       trajectory.points.at(i).pose,
//       vehicle_info_.vehicle_length_m_,
//       vehicle_info_.rear_overhang_m_);
//     const auto next_center_pose = getVehicleCenterFromBase(
//       trajectory.points.at(i + 1).pose,
//       vehicle_info_.vehicle_length_m_,
//       vehicle_info_.rear_overhang_m_);

//     Point2d prev_center_point = autoware_utils::fromMsg(prev_center_pose.position).to_2d();
//     Point2d next_center_point = autoware_utils::fromMsg(next_center_pose.position).to_2d();

//     /*
//      * create one step polygon for vehicle
//      */
//     const auto move_vehicle_polygon = createOneStepPolygon(
//       trajectory.points.at(i).pose,
//       trajectory.points.at(i + 1).pose,
//       stop_param_.expand_stop_range,
//       vehicle_info_);
//     debug_ptr_->pushPolygon(
//       move_vehicle_polygon,
//       trajectory.points.at(i).pose.position.z,
//       PolygonType::Vehicle);

//     Polygon2d move_slow_down_range_polygon;
//     if (slow_down_param_.enable_slow_down) {
//       /*
//       * create one step polygon for slow_down range
//       */
//       move_slow_down_range_polygon = createOneStepPolygon(
//         trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
//         slow_down_param_.expand_slow_down_range, vehicle_info_);
//       debug_ptr_->pushPolygon(
//         move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
//         PolygonType::SlowDownRange);
//     }

//     // check within polygon
//     pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
//       new pcl::PointCloud<pcl::PointXYZ>);
//     collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

//     std::tie(candidate_slow_down, slow_down_pointcloud_ptr) = getSlowDownPointcloud(
//       is_slow_down, slow_down_param_.enable_slow_down,
//       obstacle_candidate_pointcloud_ptr, prev_center_point, next_center_point,
//       slow_down_param_.slow_down_search_radius,
//       move_slow_down_range_polygon, slow_down_pointcloud_ptr, candidate_slow_down);

//     std::tie(is_collision, collision_pointcloud_ptr) = getCollisionPointcloud(
//       slow_down_pointcloud_ptr, prev_center_point, next_center_point,
//       stop_param_.stop_search_radius, move_vehicle_polygon,
//       trajectory.points.at(i), collision_pointcloud_ptr, is_collision);

//     if (candidate_slow_down && !is_collision && !is_slow_down) {
//       is_slow_down = true;
//       decimate_trajectory_slow_down_index = i;
//       debug_ptr_->pushPolygon(
//         move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
//         PolygonType::SlowDown);

//       const auto nearest_slow_down_pointstamped = PointHelper::getNearestPoint(
//         *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
//       nearest_slow_down_point = nearest_slow_down_pointstamped.point;
//       nearest_collision_point_time = nearest_slow_down_pointstamped.time;

//       const auto lateral_nearest_slow_down_point = PointHelper::getLateralNearestPoint(
//         *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
//       lateral_deviation = lateral_nearest_slow_down_point.deviation;
//       debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
//     }

//     /*
//      * search nearest collision point by beginning of path
//      */
//     if (is_collision) {
//       const auto nearest_collision_pointstamped = PointHelper::getNearestPoint(
//         *collision_pointcloud_ptr, trajectory.points.at(i).pose);
//       nearest_collision_point = nearest_collision_pointstamped.point;
//       nearest_collision_point_time = nearest_collision_pointstamped.time;
//       debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
//       decimate_trajectory_collision_index = i;
//       break;
//     }
//   }

//   /*
//    * insert max velocity and judge if there is a need to stop
//    */
//   bool need_to_stop = is_collision;
//   if (is_collision) {
//     std::tie(need_to_stop, output_msg) = acc_controller_->insertAdaptiveCruiseVelocity(
//       decimate_trajectory_map.decimate_trajectory,
//       decimate_trajectory_collision_index,
//       self_pose, nearest_collision_point.to_2d(),
//       nearest_collision_point_time, object_ptr_,
//       current_velocity_ptr_,
//       output_msg);
//   }

//   /*
//    * insert stop point
//    */
//   if (need_to_stop) {
//     output_msg = insertStopPoint(
//       decimate_trajectory_map.index_map.at(decimate_trajectory_collision_index) +
//       trajectory_trim_index,
//       base_path,
//       nearest_collision_point.to_2d(),
//       output_msg);
//   }

//   /*
//    * insert slow_down point
//    */
//   if (is_slow_down) {
//     output_msg = insertSlowDownPoint(
//       decimate_trajectory_map.index_map.at(decimate_trajectory_slow_down_index),
//       base_path,
//       nearest_slow_down_point.to_2d(),
//       calcSlowDownTargetVel(lateral_deviation),
//       slow_down_param_.slow_down_margin,
//       output_msg);
//   }
//   debug_ptr_->publish();
//   return output_msg;
// }

// collision
std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr>
ObstacleStopPlanner::getCollisionPointcloud(
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
ObstacleStopPlanner::getSlowDownPointcloud(
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

// autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertSlowDownPoint(
//   const size_t search_start_index,
//   const autoware_planning_msgs::msg::Trajectory & base_path,
//   const Point2d & nearest_slow_down_point,
//   const double slow_down_target_vel, const double slow_down_margin,
//   const autoware_planning_msgs::msg::Trajectory & input_msg)
// {
//   auto output_msg = input_msg;
//   PointHelper point_helper;

//   for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
//     const double yaw =
//       getYawFromQuaternion(base_path.points.at(i).pose.orientation);
//     const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
//     const Point2d slow_down_point_vec {
//       nearest_slow_down_point.x() - base_path.points.at(i).pose.position.x,
//       nearest_slow_down_point.y() - base_path.points.at(i).pose.position.y};

//     if (
//       trajectory_vec.dot(slow_down_point_vec) < 0.0 ||
//       (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(slow_down_point_vec)))
//     {
//       const auto slow_down_start_point = point_helper.createSlowDownStartPoint(
//         i, slow_down_margin, slow_down_target_vel, trajectory_vec, slow_down_point_vec,
//         base_path, current_velocity_ptr_->twist.linear.x,
//         slow_down_param_);

//       if (slow_down_start_point.index <= output_msg.points.size()) {
//         autoware_planning_msgs::msg::TrajectoryPoint slowdown_trajectory_point;
//         std::tie(slowdown_trajectory_point, output_msg) =
//           PointHelper::insertSlowDownStartPoint(
//           slow_down_start_point, base_path, output_msg);
//         debug_ptr_->pushPose(slowdown_trajectory_point.pose, PoseType::SlowDownStart);
//         output_msg = insertSlowDownVelocity(
//           slow_down_start_point.index, slow_down_target_vel, slow_down_start_point.velocity,
//           output_msg);
//       }
//       break;
//     }
//   }
//   return output_msg;
// }

// stop
autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertStopPoint(
  const size_t search_start_index,
  const autoware_planning_msgs::msg::Trajectory & base_path,
  const Point2d & nearest_collision_point,
  const autoware_planning_msgs::msg::Trajectory & input_msg)
{
  auto output_msg = input_msg;
  PointHelper point_helper;

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
        point_helper.searchInsertPoint(
        i, base_path, trajectory_vec, collision_point_vec, stop_param_);
      if (stop_point.index <= output_msg.points.size()) {
        autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
        std::tie(trajectory_point, output_msg) =
          PointHelper::insertStopPoint(stop_point, base_path, output_msg);
        debug_ptr_->pushPose(trajectory_point.pose, PoseType::Stop);
      }
      break;
    }
  }
  return output_msg;
}

autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertSlowDownVelocity(
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
          slow_down_vel * slow_down_vel - 2 * slow_down_param_.max_deceleration * dist,
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

double ObstacleStopPlanner::calcSlowDownTargetVel(const double lateral_deviation) const
{
  return slow_down_param_.min_slow_down_vel +
         (slow_down_param_.max_slow_down_vel - slow_down_param_.min_slow_down_vel) *
         std::max(lateral_deviation - vehicle_info_.vehicle_width_m / 2, 0.0) /
         slow_down_param_.expand_slow_down_range;
}

}  // namespace obstacle_stop_planner
