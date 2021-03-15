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

#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "obstacle_stop_planner/obstacle_stop_planner.hpp"
#include "obstacle_stop_planner/util/util.hpp"
#include "obstacle_stop_planner/util/one_step_polygon.hpp"
#include "obstacle_stop_planner/util/trajectory.hpp"
#include "obstacle_stop_planner/util/obstacle_point_cloud.hpp"
#include "obstacle_stop_planner/util/point_helper.hpp"
#include "obstacle_stop_planner/control/slow_down_control.hpp"
#include "obstacle_stop_planner/control/stop_control.hpp"

#define EIGEN_MPL2_ONLY
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace obstacle_stop_planner
{
ObstacleStopPlanner::ObstacleStopPlanner(
  const VehicleInfo & vehicle_info,
  const StopControlParameter & stop_param,
  const SlowDownControlParameter & slow_down_param,
  const AdaptiveCruiseControlParameter & acc_param)
: vehicle_info_(vehicle_info),
  stop_param_(stop_param),
  slow_down_param_(slow_down_param),
  acc_param_(acc_param)
{
  // debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(
  //   this,
  //   vehicle_info.wheel_base +
  //   vehicle_info.front_overhang);

  // Initializer
  acc_controller_ = std::make_unique<obstacle_stop_planner::AdaptiveCruiseController>(
    vehicle_info.vehicle_width, vehicle_info.vehicle_length,
    vehicle_info.wheel_base, vehicle_info.front_overhang);

  obstacle_pointcloud_ = std::make_shared<ObstaclePointCloud>();
}

ObstacleStopPlanner::~ObstacleStopPlanner() = default;

void ObstacleStopPlanner::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
{
  obstacle_pointcloud_->updatePointCloud(pc);
}

TrajectorySet ObstacleStopPlanner::processTrajectory(
  const Trajectory & input_path,
  const Pose & self_pose)
{
  TrajectorySet output;
  output.orig = input_path;
  /*
   * trim trajectory from self pose
   */
  std::tie(output.trim, output.trimmed_idx) =
    trimTrajectoryWithIndexFromSelfPose(input_path, self_pose);

  /*
   * decimate trajectory for calculation cost
   */
  DecimateTrajectoryMap decimate_trajectory_map = decimateTrajectory(
    output.trim, stop_param_.step_length);

  output.decimate = decimate_trajectory_map.decimate;
  output.decimated_idx_map = decimate_trajectory_map.index_map;

  return output;
}

Trajectory ObstacleStopPlanner::updatePath(
  const TrajectorySet & input_path,
  const Pose & self_pose,
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  const Trajectory base_path = input_path.orig;
  const Trajectory trajectory = input_path.decimate;

  /*
   * search candidate obstacle pointcloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  // search obstacle candidate pointcloud to reduce calculation cost
  const auto search_radius = slow_down_param_.enable_slow_down ?
    slow_down_param_.slow_down_search_radius : stop_param_.stop_search_radius;

  obstacle_pointcloud_->searchCandidateObstacle(
    transform_stamped,
    trajectory,
    search_radius,
    vehicle_info_);

  stop_control_->clear();
  slow_down_control_->clear();

  /*
   * check collision, slow_down
   */
  bool is_slow_down = false;
  size_t decimate_trajectory_collision_index;
  Point2d nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  size_t decimate_trajectory_slow_down_index;
  Point2d nearest_slow_down_point;

  // for collision
  // bool is_collision = false;
  // for slow down
  // bool candidate_slow_down = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  double lateral_deviation = 0.0;
  PointHelper point_helper;

  for (int i = 0; i < static_cast<int>(trajectory.points.size()) - 1; ++i) {
    /*
     * create one step circle center for vehicle
     */
    const auto prev_center_point = getVehicleCenterFromBase(
      trajectory.points.at(i).pose,
      vehicle_info_.vehicle_length,
      vehicle_info_.rear_overhang);
    const auto next_center_point = getVehicleCenterFromBase(
      trajectory.points.at(i + 1).pose,
      vehicle_info_.vehicle_length,
      vehicle_info_.rear_overhang);
    const Point2dPair vehicle_center_points {prev_center_point, next_center_point};

    /*
     * create one step polygon for vehicle
     */
    const auto move_vehicle_polygon = stop_control_->createVehiclePolygon(
      trajectory.points.at(i).pose,
      trajectory.points.at(i + 1).pose,
      vehicle_info_);
    // const auto move_vehicle_polygon = createOneStepPolygon(
    //   trajectory.points.at(i).pose,
    //   trajectory.points.at(i + 1).pose,
    //   param_.expand_stop_range,
    //   param_.vehicle_info);
    // debug_ptr_->pushPolygon(
    //   move_vehicle_polygon,
    //   trajectory.points.at(i).pose.position.z,
    //   PolygonType::Vehicle);

    /*
    * create one step polygon for slow_down range
    */
    const auto move_slow_down_range_polygon = slow_down_control_->createVehiclePolygon(
      trajectory.points.at(i).pose,
      trajectory.points.at(i + 1).pose,
      vehicle_info_);
    // Polygon2d move_slow_down_range_polygon;
    // if (param_.enable_slow_down) {
    //   /*
    //   * create one step polygon for slow_down range
    //   */
    //   move_slow_down_range_polygon = createOneStepPolygon(
    //     trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose,
    //     param_.expand_slow_down_range, param_.vehicle_info);
    //   debug_ptr_->pushPolygon(
    //     move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
    //     PolygonType::SlowDownRange);
    // }

    // check within polygon

    // std::tie(candidate_slow_down, slow_down_pointcloud_ptr) = getSlowDownPointcloud(
    //   is_slow_down, param_.enable_slow_down,
    //   obstacle_candidate_pointcloud_ptr, prev_center_point, next_center_point,
    //   param_.slow_down_search_radius,
    //   move_slow_down_range_polygon, slow_down_pointcloud_ptr, candidate_slow_down);
    slow_down_pointcloud_ptr = slow_down_control_->getSlowDownPointcloud(
      is_slow_down,
      obstacle_candidate_pointcloud_ptr,
      vehicle_center_points,
      move_slow_down_range_polygon);

    // std::tie(is_collision, collision_pointcloud_ptr) = getCollisionPointcloud(
    //   slow_down_pointcloud_ptr, prev_center_point, next_center_point,
    //   param_.stop_search_radius, move_vehicle_polygon,
    //   trajectory.points.at(i), collision_pointcloud_ptr, is_collision);
    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

    collision_pointcloud_ptr = stop_control_->getCollisionPointcloud(
      slow_down_pointcloud_ptr,
      vehicle_center_points,
      move_vehicle_polygon,
      collision_pointcloud_ptr);

    // if (slow_down_control_->candidateSlowDown()
    //     && !stop_control->is_collision()
    //     && !is_slow_down) {
    //   is_slow_down = true;
    //   decimate_trajectory_slow_down_index = i;  // ???
    //   // debug_ptr_->pushPolygon(
    //   //   move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
    //   //   PolygonType::SlowDown);

    //   const auto nearest_slow_down_pointstamped = point_helper.getNearestPoint(
    //     *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
    //   nearest_slow_down_point = nearest_slow_down_pointstamped.point;
    //   nearest_collision_point_time = nearest_slow_down_pointstamped.time;

    //   const auto lateral_nearest_slow_down_point = point_helper.getLateralNearestPoint(
    //     *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
    //   lateral_deviation = lateral_nearest_slow_down_point.deviation;
    //   // debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
    // }

    // /*
    //  * search nearest collision point by beginning of path
    //  */
    // if (stop_control->is_collision()) {
    //   const auto nearest_collision_pointstamped = point_helper.getNearestPoint(
    //     *collision_pointcloud_ptr, trajectory.points.at(i).pose);
    //   nearest_collision_point = nearest_collision_pointstamped.point;
    //   nearest_collision_point_time = nearest_collision_pointstamped.time;
    //   // debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
    //   decimate_trajectory_collision_index = i;
    //   break;
    // }

    if (stop_control_->isCollision()) {
      /*
      * search nearest collision point by beginning of path
      */
      const auto nearest_collision_pointstamped = point_helper.getNearestPoint(
        *collision_pointcloud_ptr, trajectory.points.at(i).pose);
      nearest_collision_point = nearest_collision_pointstamped.point;
      nearest_collision_point_time = nearest_collision_pointstamped.time;
      // debug_ptr_->pushObstaclePoint(nearest_collision_point, PointType::Stop);
      decimate_trajectory_collision_index = i;
      break;
    } else {
      if (slow_down_control_->candidateSlowDown() && !is_slow_down) {
        is_slow_down = true;
        decimate_trajectory_slow_down_index = i;  // ???
        // debug_ptr_->pushPolygon(
        //   move_slow_down_range_polygon, trajectory.points.at(i).pose.position.z,
        //   PolygonType::SlowDown);

        const auto nearest_slow_down_pointstamped = point_helper.getNearestPoint(
          *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
        nearest_slow_down_point = nearest_slow_down_pointstamped.point;
        nearest_collision_point_time = nearest_slow_down_pointstamped.time;

        // FIXME: replace getLateralDeviation()
        const auto lateral_nearest_slow_down_point = point_helper.getLateralNearestPoint(
          *slow_down_pointcloud_ptr, trajectory.points.at(i).pose);
        lateral_deviation = lateral_nearest_slow_down_point.deviation;
        // debug_ptr_->pushObstaclePoint(nearest_slow_down_point, PointType::SlowDown);
      }
    }
  }

  // Create output path
  Trajectory output_msg = input_path.orig;

  /*
   * insert max velocity and judge if there is a need to stop
   */
  bool need_to_stop;
  if (stop_control_->isCollision()) {
    std::tie(need_to_stop, output_msg) = acc_controller_->insertAdaptiveCruiseVelocity(
      input_path.decimate,
      decimate_trajectory_collision_index,
      self_pose, nearest_collision_point,
      nearest_collision_point_time, object_ptr_,
      current_velocity_ptr_,
      output_msg);
  } else {
    need_to_stop = stop_control_->isCollision();
  }

  /*
   * insert stop point
   */
  if (need_to_stop) {
    output_msg = stop_control_->insertStopPoint(
      input_path.decimated_idx_map.at(decimate_trajectory_collision_index) +
      input_path.trimmed_idx,
      base_path,
      nearest_collision_point,
      output_msg);
  }

  /*
   * insert slow_down point
   */
  if (is_slow_down) {
    output_msg = slow_down_control_->insertSlowDownPoint(
      input_path.decimated_idx_map.at(decimate_trajectory_slow_down_index),
      base_path,
      nearest_slow_down_point,
      lateral_deviation,
      current_velocity_ptr_->twist.angular.x,
      vehicle_info_.vehicle_width,
      output_msg);
  }
  // path_pub_->publish(output_msg);
  // debug_ptr_->publish();

  return output_msg;
}

// collision
// std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr>
// ObstacleStopPlanner::getCollisionPointcloud(
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
//   const Point2dPair & vehicle_center_points,
//   const double search_radius,
//   const Polygon2d & one_step_polygon,
//   const autoware_planning_msgs::msg::TrajectoryPoint & trajectory_point,
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud,
//   const bool is_collision)
// {
//   auto output_pointcloud = collision_pointcloud;
//   auto output_collision = is_collision;

//   for (size_t j = 0; j < slow_down_pointcloud->size(); ++j) {
//     Point2d point(slow_down_pointcloud->at(j).x, slow_down_pointcloud->at(j).y);
//     if (
//       boost::geometry::distance(vehicle_center_points.prev, point) < search_radius ||
//       boost::geometry::distance(vehicle_center_points.next, point) < search_radius)
//     {
//       if (boost::geometry::within(point, one_step_polygon)) {
//         output_pointcloud->push_back(slow_down_pointcloud->at(j));
//         output_collision = true;
//         debug_ptr_->pushPolygon(
//           one_step_polygon, trajectory_point.pose.position.z,
//           PolygonType::Collision);
//       }
//     }
//   }
//   return std::make_tuple(output_collision, output_pointcloud);
// }

// slow down
// std::tuple<bool, pcl::PointCloud<pcl::PointXYZ>::Ptr>
// ObstacleStopPlanner::getSlowDownPointcloud(
//   const bool is_slow_down,
//   const bool enable_slow_down,
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud,
//   const Point2d & prev_center_point,
//   const Point2d & next_center_point,
//   const double search_radius,
//   const Polygon2d & one_step_polygon,
//   const pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud,
//   const bool candidate_slow_down)
// {
//   auto output_pointcloud = slow_down_pointcloud;
//   auto output_candidate = candidate_slow_down;

//   if (!is_slow_down && enable_slow_down) {
//     for (size_t j = 0; j < obstacle_candidate_pointcloud->size(); ++j) {
//       Point2d point(
//         obstacle_candidate_pointcloud->at(j).x, obstacle_candidate_pointcloud->at(j).y);
//       if (
//         boost::geometry::distance(prev_center_point, point) < search_radius ||
//         boost::geometry::distance(next_center_point, point) < search_radius)
//       {
//         if (boost::geometry::within(point, one_step_polygon)) {
//           output_pointcloud->push_back(obstacle_candidate_pointcloud->at(j));
//           output_candidate = true;
//         }
//       }
//     }
//   } else {
//     output_pointcloud = obstacle_candidate_pointcloud;
//   }
//   return std::make_tuple(output_candidate, output_pointcloud);
// }

// autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertSlowDownPoint(
//   const size_t search_start_index,
//   const autoware_planning_msgs::msg::Trajectory & base_path,
//   const Point2d & nearest_slow_down_point,
//   const double slow_down_target_vel, const double slow_down_margin,
//   const autoware_planning_msgs::msg::Trajectory & input_msg)
// {
//   auto output_msg = input_msg;
//   PointHelper point_helper {param_};

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
//         base_path, current_velocity_ptr_->twist.linear.x);

//       if (slow_down_start_point.index <= output_msg.points.size()) {
//         autoware_planning_msgs::msg::TrajectoryPoint slowdown_trajectory_point;
//         std::tie(slowdown_trajectory_point, output_msg) =
//           point_helper.insertSlowDownStartPoint(
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
// autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertStopPoint(
//   const size_t search_start_index,
//   const autoware_planning_msgs::msg::Trajectory & base_path,
//   const Point2d & nearest_collision_point,
//   const autoware_planning_msgs::msg::Trajectory & input_msg)
// {
//   auto output_msg = input_msg;
//   PointHelper point_helper {param_};

//   for (size_t i = search_start_index; i < base_path.points.size(); ++i) {
//     const double yaw =
//       getYawFromQuaternion(base_path.points.at(i).pose.orientation);
//     const Point2d trajectory_vec {std::cos(yaw), std::sin(yaw)};
//     const Point2d collision_point_vec {
//       nearest_collision_point.x() - base_path.points.at(i).pose.position.x,
//       nearest_collision_point.y() - base_path.points.at(i).pose.position.y};

//     if (
//       trajectory_vec.dot(collision_point_vec) < 0.0 ||
//       (i + 1 == base_path.points.size() && 0.0 < trajectory_vec.dot(collision_point_vec)))
//     {
//       const auto stop_point =
//         point_helper.searchInsertPoint(i, base_path, trajectory_vec, collision_point_vec);
//       if (stop_point.index <= output_msg.points.size()) {
//         autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
//         std::tie(trajectory_point, output_msg) =
//           point_helper.insertStopPoint(stop_point, base_path, output_msg);
//         debug_ptr_->pushPose(trajectory_point.pose, PoseType::Stop);
//       }
//       break;
//     }
//   }
//   return output_msg;
// }

// autoware_planning_msgs::msg::Trajectory ObstacleStopPlanner::insertSlowDownVelocity(
//   const size_t slow_down_start_point_idx, const double slow_down_target_vel, double slow_down_vel,
//   const autoware_planning_msgs::msg::Trajectory & input_path)
// {
//   autoware_planning_msgs::msg::TrajectoryPoint slow_down_end_trajectory_point;
//   auto output_path = input_path;
//   bool is_slow_down_end = false;

//   for (size_t j = slow_down_start_point_idx; j < output_path.points.size() - 1; ++j) {
//     output_path.points.at(j).twist.linear.x =
//       std::min(slow_down_vel, output_path.points.at(j).twist.linear.x);
//     const auto dist = std::hypot(
//       output_path.points.at(j).pose.position.x - output_path.points.at(j + 1).pose.position.x,
//       output_path.points.at(j).pose.position.y - output_path.points.at(j + 1).pose.position.y);
//     slow_down_vel = std::max(
//       slow_down_target_vel,
//       std::sqrt(
//         std::max(
//           slow_down_vel * slow_down_vel - 2 * param_.max_deceleration * dist,
//           0.0)));
//     if (!is_slow_down_end && slow_down_vel <= slow_down_target_vel) {
//       slow_down_end_trajectory_point = output_path.points.at(j + 1);
//       is_slow_down_end = true;
//     }
//   }
//   if (!is_slow_down_end) {
//     slow_down_end_trajectory_point = output_path.points.back();
//   }
//   debug_ptr_->pushPose(slow_down_end_trajectory_point.pose, PoseType::SlowDownEnd);
//   return output_path;
// }

// double ObstacleStopPlanner::calcSlowDownTargetVel(const double lateral_deviation)
// {
//   return param_.min_slow_down_vel +
//          (param_.max_slow_down_vel - param_.min_slow_down_vel) *
//          std::max(lateral_deviation - param_.vehicle_info.vehicle_width / 2, 0.0) /
//          param_.expand_slow_down_range;
// }

}  // namespace obstacle_stop_planner
