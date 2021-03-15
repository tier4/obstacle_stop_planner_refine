// Copyright 2019 Autoware Foundation
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
#ifndef OBSTACLE_STOP_PLANNER__NODE_HPP_
#define OBSTACLE_STOP_PLANNER__NODE_HPP_

#include <map>
#include <memory>
#include <vector>
#include <tuple>

#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/dynamic_object_array.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "autoware_debug_msgs/msg/float32_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "obstacle_stop_planner/debug_marker.hpp"
#include "obstacle_stop_planner/util/util.hpp"
#include "obstacle_stop_planner/control/adaptive_cruise_control.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "obstacle_stop_planner/parameter/adaptive_cruise_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/slow_down_control_parameter.hpp"
#include "obstacle_stop_planner/parameter/stop_control_parameter.hpp"

namespace obstacle_stop_planner
{

using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;

class ObstaclePointCloud;
class StopController;
class SlowDownController;

struct OBSTACLE_STOP_PLANNER_PUBLIC TrajectorySet
{
  Trajectory orig;
  Trajectory trim;
  Trajectory decimate;
  size_t trimmed_idx;
  std::map<size_t /* decimate */, size_t /* origin */> decimated_idx_map;
};

// MEMO: 役割　経路の間引きや自己位置の取得、ポリゴンの作成
class OBSTACLE_STOP_PLANNER_PUBLIC ObstacleStopPlanner
{
public:
  explicit ObstacleStopPlanner(
    const VehicleInfo & vehicle_info,
    const StopControlParameter & stop_param,
    const SlowDownControlParameter & slow_down_param,
    const AdaptiveCruiseControlParameter & acc_param);
  ~ObstacleStopPlanner();

  TrajectorySet processTrajectory(
    const Trajectory & input_path,
    const Pose & self_pose);

  Trajectory updatePath(
    const TrajectorySet & input_path,
    const Pose & self_pose,
    const geometry_msgs::msg::TransformStamped & transform_stamped);

  void updatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void updateExpandStopRange(const double expand_stop_range);

private:
  /*
   * ROS
   */
  std::shared_ptr<ObstacleStopPlannerDebugNode> debug_ptr_;
  std::shared_ptr<ObstaclePointCloud> obstacle_pointcloud_;
  VehicleInfo vehicle_info_;
  StopControlParameter stop_param_;
  SlowDownControlParameter slow_down_param_;
  AdaptiveCruiseControlParameter acc_param_;

  /*
   * Parameter
   */
  std::unique_ptr<obstacle_stop_planner::AdaptiveCruiseController> acc_controller_;
  std::unique_ptr<obstacle_stop_planner::StopController> stop_control_;
  std::unique_ptr<obstacle_stop_planner::SlowDownController> slow_down_control_;
  // rclcpp::Time prev_col_point_time_;
  // pcl::PointXYZ prev_col_point_;
  geometry_msgs::msg::TwistStamped::SharedPtr current_velocity_ptr_;
  autoware_perception_msgs::msg::DynamicObjectArray::SharedPtr object_ptr_;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__NODE_HPP_
