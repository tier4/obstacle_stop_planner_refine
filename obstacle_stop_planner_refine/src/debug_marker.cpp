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
#include <vector>

#include "obstacle_stop_planner/debug_marker.hpp"
#include "autoware_utils/autoware_utils.hpp"

namespace obstacle_stop_planner
{
ObstacleStopPlannerDebugNode::ObstacleStopPlannerDebugNode(
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & node_clock, const double base_link2front)
: node_clock_(node_clock), base_link2front_(base_link2front)
{
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const Polygon2d & polygon, const double z, const PolygonType & type)
{
  Polygon3d polygon3d;
  for (const auto & point : polygon.outer()) {
    polygon3d.outer().emplace_back(point.to_3d(z));
  }
  return pushPolygon(polygon3d, type);
}

void ObstacleStopPlannerDebugNode::pushPolygons(
  const std::vector<Polygon2d> & polygons, const std::vector<double> & z, const PolygonType & type)
{
  for (size_t i = 0; i < polygons.size(); ++i) {
    pushPolygon(polygons.at(i), z.at(i), type);
  }
}

bool ObstacleStopPlannerDebugNode::pushPolygon(
  const Polygon3d & polygon, const PolygonType & type)
{
  switch (type) {
    case PolygonType::Vehicle:
      if (!polygon.outer().empty()) {vehicle_polygons_.emplace_back(polygon);}
      return true;
    case PolygonType::Collision:
      if (!polygon.outer().empty()) {collision_polygons_.emplace_back(polygon);}
      return true;
    case PolygonType::SlowDownRange:
      if (!polygon.outer().empty()) {slow_down_range_polygons_.emplace_back(polygon);}
      return true;
    case PolygonType::SlowDown:
      if (!polygon.outer().empty()) {slow_down_polygons_.emplace_back(polygon);}
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose,
  const PoseType & type)
{
  switch (type) {
    case PoseType::Stop:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    case PoseType::SlowDownStart:
      slow_down_start_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    case PoseType::SlowDownEnd:
      slow_down_end_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::Stop:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    case PointType::SlowDown:
      slow_down_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

bool ObstacleStopPlannerDebugNode::pushObstaclePoint(
  const Point3d & obstacle_point, const PointType & type)
{
  return pushObstaclePoint(autoware_utils::toMsg(obstacle_point), type);
}

visualization_msgs::msg::MarkerArray ObstacleStopPlannerDebugNode::makeVisualizationMarker()
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = node_clock_->get_clock()->now();
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front_, 0.0, 0.0));

  // polygon
  if (!vehicle_polygons_.empty()) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "detection_polygons", 0,
      visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (auto & vehicle_polygon : vehicle_polygons_) {
      for (size_t j = 0; j < vehicle_polygon.outer().size(); ++j) {
        {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            vehicle_polygon.outer().at(
              j));
          marker.points.push_back(point);
        }
        if (j + 1 == vehicle_polygon.outer().size()) {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            vehicle_polygon.outer().at(
              0));
          marker.points.push_back(point);
        } else {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            vehicle_polygon.outer().at(
              j + 1));
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!collision_polygons_.empty()) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "collision_polygons", 0,
      visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils::createMarkerScale(0.05, 0.0, 0.0),
      autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));

    for (auto & collision_polygon : collision_polygons_) {
      for (size_t j = 0; j < collision_polygon.outer().size(); ++j) {
        {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            collision_polygon.outer().at(j));
          marker.points.push_back(point);
        }
        if (j + 1 == collision_polygon.outer().size()) {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            collision_polygon.outer().at(0));
          marker.points.push_back(point);

        } else {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            collision_polygon.outer().at(j + 1));
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_range_polygons_.empty()) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "slow_down_detection_polygons", 0,
      visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (auto & slow_down_range_polygon : slow_down_range_polygons_) {
      for (size_t j = 0; j < slow_down_range_polygon.outer().size(); ++j) {
        {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_range_polygon.outer().at(j));
          marker.points.push_back(point);
        }
        if (j + 1 == slow_down_range_polygon.outer().size()) {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_range_polygon.outer().at(0));
          marker.points.push_back(point);

        } else {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_range_polygon.outer().at(j + 1));
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (!slow_down_polygons_.empty()) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "slow_down_polygons", 0,
      visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils::createMarkerScale(0.05, 0.0, 0.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (auto & slow_down_polygon : slow_down_polygons_) {
      for (size_t j = 0; j < slow_down_polygon.outer().size(); ++j) {
        {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_polygon.outer().at(j));
          marker.points.push_back(point);
        }
        if (j + 1 == slow_down_polygon.outer().size()) {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_polygon.outer().at(0));
          marker.points.push_back(point);

        } else {
          geometry_msgs::msg::Point point = autoware_utils::toMsg(
            slow_down_polygon.outer().at(j + 1));
          marker.points.push_back(point);
        }
      }
    }
    msg.markers.push_back(marker);
  }

  if (stop_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "virtual_wall/stop", 0,
      visualization_msgs::msg::Marker::CUBE,
      autoware_utils::createMarkerScale(0.1, 5.0, 2.0),
      autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.5));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;

    msg.markers.push_back(marker);
  }

  if (stop_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "factor_text/stop", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.text = "obstacle";

    msg.markers.push_back(marker);
  }

  if (slow_down_start_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "virtual_wall/slow_down_start", 0,
      visualization_msgs::msg::Marker::CUBE,
      autoware_utils::createMarkerScale(0.1, 5.0, 2.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 0.5));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*slow_down_start_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;

    msg.markers.push_back(marker);
  }

  if (slow_down_start_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "factor_text/slow_down_start", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*slow_down_start_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.text = "slow down\nstart";

    msg.markers.push_back(marker);
  }

  if (slow_down_end_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "virtual_wall/slow_down_end", 0,
      visualization_msgs::msg::Marker::CUBE,
      autoware_utils::createMarkerScale(0.1, 5.0, 2.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 0.5));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*slow_down_end_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;

    msg.markers.push_back(marker);
  }

  if (slow_down_end_pose_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "factor_text/slow_down_end", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999));

    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*slow_down_end_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.text = "slow down\nend";

    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "stop_obstacle_point", 0,
      visualization_msgs::msg::Marker::SPHERE,
      autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = *stop_obstacle_point_ptr_;

    msg.markers.push_back(marker);
  }

  if (stop_obstacle_point_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "stop_obstacle_text", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.text = "!";

    msg.markers.push_back(marker);
  }

  if (slow_down_obstacle_point_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "slow_down_obstacle_point", 0,
      visualization_msgs::msg::Marker::SPHERE,
      autoware_utils::createMarkerScale(0.25, 0.25, 0.25),
      autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = *slow_down_obstacle_point_ptr_;

    msg.markers.push_back(marker);
  }

  if (slow_down_obstacle_point_ptr_ != nullptr) {
    auto marker = autoware_utils::createDefaultMarker(
      "map", current_time, "slow_down_obstacle_text", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::createMarkerScale(0.0, 0.0, 1.0),
      autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = *slow_down_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;
    marker.text = "!";

    msg.markers.push_back(marker);
  }

  return msg;
}

autoware_planning_msgs::msg::StopReasonArray ObstacleStopPlannerDebugNode::makeStopReasonArray()
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = node_clock_->get_clock()->now();

  // create stop reason stamped
  autoware_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = autoware_planning_msgs::msg::StopReason::OBSTACLE_STOP;
  autoware_planning_msgs::msg::StopFactor stop_factor;

  if (stop_pose_ptr_ != nullptr) {
    stop_factor.stop_pose = *stop_pose_ptr_;
    if (stop_obstacle_point_ptr_ != nullptr) {
      stop_factor.stop_factor_points.emplace_back(*stop_obstacle_point_ptr_);
    }
    stop_reason_msg.stop_factors.emplace_back(stop_factor);
  }

  // create stop reason array
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

}  // namespace obstacle_stop_planner
