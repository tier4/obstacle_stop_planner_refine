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

  stop_control_ = std::make_unique<obstacle_stop_planner::StopController>(stop_param_);
  slow_down_control_ =
    std::make_unique<obstacle_stop_planner::SlowDownController>(slow_down_param_);

  obstacle_pointcloud_ = std::make_shared<ObstaclePointCloud>();
}

ObstacleStopPlanner::~ObstacleStopPlanner() = default;

void ObstacleStopPlanner::updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
{
  obstacle_pointcloud_->updatePointCloud(pc);
}

void ObstacleStopPlanner::updateExpandStopRange(const double expand_stop_range)
{
  stop_param_.expand_stop_range = expand_stop_range;
  stop_param_.stop_search_radius =
    stop_param_.step_length + std::hypot(
    vehicle_info_.vehicle_width / 2.0 + stop_param_.expand_stop_range,
    vehicle_info_.vehicle_length / 2.0);
  stop_control_->updateParameter(stop_param_);
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
  const geometry_msgs::msg::TransformStamped & transform_stamped,
  const double current_velocity,
  const rclcpp::Time & current_time)
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

  obstacle_candidate_pointcloud_ptr = obstacle_pointcloud_->searchCandidateObstacle(
    transform_stamped,
    trajectory,
    search_radius,
    vehicle_info_);

  stop_control_->clear();
  slow_down_control_->clear();

  /*
   * check collision, slow_down
   */
  // for collision
  size_t decimate_trajectory_collision_index;
  Point2d nearest_collision_point;
  rclcpp::Time nearest_collision_point_time;
  // for slow down
  bool is_slow_down = false;
  size_t decimate_trajectory_slow_down_index;
  Point2d nearest_slow_down_point;

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

    slow_down_pointcloud_ptr = slow_down_control_->getSlowDownPointcloud(
      is_slow_down,
      obstacle_candidate_pointcloud_ptr,
      vehicle_center_points,
      move_slow_down_range_polygon);

    pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

    collision_pointcloud_ptr = stop_control_->getCollisionPointcloud(
      slow_down_pointcloud_ptr,
      vehicle_center_points,
      move_vehicle_polygon,
      collision_pointcloud_ptr);

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
      current_velocity,
      output_msg,
      current_time);
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
      current_velocity,
      vehicle_info_.vehicle_width,
      output_msg);
  }
  // debug_ptr_->publish();

  return output_msg;
}

}  // namespace obstacle_stop_planner
