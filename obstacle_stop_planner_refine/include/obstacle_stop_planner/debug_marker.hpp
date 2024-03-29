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
#ifndef OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
#define OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "autoware_planning_msgs/msg/stop_reason_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "obstacle_stop_planner/util.hpp"

namespace obstacle_stop_planner
{
enum class PolygonType : int8_t { Vehicle = 0, Collision, SlowDownRange, SlowDown };
enum class PointType : int8_t { Stop = 0, SlowDown };
enum class PoseType : int8_t { Stop = 0, SlowDownStart, SlowDownEnd };

class ObstacleStopPlannerDebugNode
{
public:
  explicit ObstacleStopPlannerDebugNode(rclcpp::Node * node, const double base_link2front);
  ~ObstacleStopPlannerDebugNode() {}
  bool pushPolygon(
    const Polygon2d & polygon, const double z, const PolygonType & type);
  bool pushPolygon(const Polygon3d & polygon, const PolygonType & type);
  bool pushPose(const geometry_msgs::msg::Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const geometry_msgs::msg::Point & obstacle_point, const PointType & type);
  bool pushObstaclePoint(const Point3d & obstacle_point, const PointType & type);
  visualization_msgs::msg::MarkerArray makeVisualizationMarker();
  autoware_planning_msgs::msg::StopReasonArray makeStopReasonArray();

  void publish();

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr stop_reason_pub_;
  rclcpp::Node * node_;
  double base_link2front_;

  std::shared_ptr<geometry_msgs::msg::Pose> stop_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> slow_down_start_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> slow_down_end_pose_ptr_;
  std::shared_ptr<geometry_msgs::msg::Point> stop_obstacle_point_ptr_;
  std::shared_ptr<geometry_msgs::msg::Point> slow_down_obstacle_point_ptr_;
  std::vector<Polygon3d> vehicle_polygons_;
  std::vector<Polygon3d> slow_down_range_polygons_;
  std::vector<Polygon3d> collision_polygons_;
  std::vector<Polygon3d> slow_down_polygons_;
};

}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
