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

#ifndef OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_
#define OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_

#include <vector>
#include <memory>
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacle_stop_planner/util.hpp"
#include "obstacle_stop_planner/vehicle.hpp"


namespace motion_planning {

class OneStepPolygon
{
public:
  void Create(
    const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
    const double expand_width);
  Polygon GetBoostPolygon() const {return boost_polygon_;}
  std::vector<cv::Point2d> GetPolygon() const {return polygon_;}
  void SetVehicleInfo(VehicleInfo vehicle_info) {vehicle_info_ = std::make_shared<VehicleInfo>(vehicle_info);}

private:
  std::vector<cv::Point2d> polygon_;
  Polygon boost_polygon_;
  std::shared_ptr<VehicleInfo> vehicle_info_;

  Polygon ConvertToBoostPolygon(const std::vector<cv::Point2d>& polygon);
  std::vector<cv::Point2d> convexHull(const std::vector<cv::Point2d> pointcloud);
};

inline Polygon OneStepPolygon::ConvertToBoostPolygon(const std::vector<cv::Point2d>& polygon)
{
  // convert boost polygon
  Polygon boost_polygon;
  for (const auto & point : polygon) {
    boost_polygon.outer().push_back(bg::make<Point>(point.x, point.y));
  }
  boost_polygon.outer().push_back(
    bg::make<Point>(
      polygon.front().x, polygon.front().y));
  return boost_polygon;
}

inline void OneStepPolygon::Create(
  const geometry_msgs::msg::Pose base_step_pose, const geometry_msgs::msg::Pose next_step_pose,
  const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;
  // start step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(base_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) -
        std::sin(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) +
        std::cos(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-vehicle_info_->rear_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-vehicle_info_->rear_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        base_step_pose.position.x + std::cos(yaw) * (-vehicle_info_->rear_overhang_m_) -
        std::sin(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width),
        base_step_pose.position.y + std::sin(yaw) * (-vehicle_info_->rear_overhang_m_) +
        std::cos(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width)));
  }
  // next step
  {
    double yaw = getYawFromGeometryMsgsQuaternion(next_step_pose.orientation);
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) -
        std::sin(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) +
        std::cos(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * (vehicle_info_->wheel_base_m_ + vehicle_info_->front_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-vehicle_info_->rear_overhang_m_) -
        std::sin(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-vehicle_info_->rear_overhang_m_) +
        std::cos(yaw) * (-vehicle_info_->vehicle_width_m_ / 2.0 - expand_width)));
    one_step_move_vehicle_corner_points.push_back(
      cv::Point2d(
        next_step_pose.position.x + std::cos(yaw) * (-vehicle_info_->rear_overhang_m_) -
        std::sin(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width),
        next_step_pose.position.y + std::sin(yaw) * (-vehicle_info_->rear_overhang_m_) +
        std::cos(yaw) * (vehicle_info_->vehicle_width_m_ / 2.0 + expand_width)));
  }
  polygon_ = convexHull(one_step_move_vehicle_corner_points);
  boost_polygon_ = ConvertToBoostPolygon(polygon_);
}

inline std::vector<cv::Point2d> OneStepPolygon::convexHull(
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

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__ONE_STEP_POLYGON_HPP_
