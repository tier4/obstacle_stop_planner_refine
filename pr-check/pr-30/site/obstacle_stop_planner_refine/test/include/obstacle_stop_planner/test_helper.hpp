// Copyright 2021 Tier IV, Inc
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
#ifndef OBSTACLE_STOP_PLANNER__TEST_HELPER_HPP_
#define OBSTACLE_STOP_PLANNER__TEST_HELPER_HPP_

#include <vector>
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_utils/autoware_utils.hpp"

using autoware_utils::Point3d;
using autoware_planning_msgs::msg::Trajectory;

Trajectory convertPointsToTrajectoryWithYaw(
  const std::vector<Point3d> & points)
{
  Trajectory trajectory;
  for (size_t i = 0; i < points.size(); i++) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose.position = autoware_utils::toMsg(points[i]);
    double yaw = 0;
    if (i > 0) {
      const double dx = points[i].x() - points[i - 1].x();
      const double dy = points[i].y() - points[i - 1].y();
      yaw = std::atan2(dy, dx);
    } else if (i == 0 && points.size() > 1) {
      const double dx = points[i + 1].x() - points[i].x();
      const double dy = points[i + 1].y() - points[i].y();
      yaw = std::atan2(dy, dx);
    }
    const double roll = 0;
    const double pitch = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    trajectory.points.push_back(traj_point);
  }
  return trajectory;
}

#endif  // OBSTACLE_STOP_PLANNER__TEST_HELPER_HPP_
