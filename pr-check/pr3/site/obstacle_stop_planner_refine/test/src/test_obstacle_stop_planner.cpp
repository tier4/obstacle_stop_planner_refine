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

#include <memory>
#include <vector>
#include <string>
#include "gtest/gtest.h"

#include "obstacle_stop_planner/obstacle_stop_planner.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using obstacle_stop_planner::StopControlParameter;
using obstacle_stop_planner::SlowDownControlParameter;
using obstacle_stop_planner::AdaptiveCruiseControlParameter;
using obstacle_stop_planner::ObstacleStopPlanner;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

VehicleInfo createVehicleParam()
{
  VehicleInfo i;
  i.wheel_radius = 1.0F;
  i.wheel_width = 1.0F;
  i.wheel_base = 1.0F;
  i.wheel_tread = 1.0F;
  i.front_overhang = 1.0F;
  i.rear_overhang = 1.0F;
  i.left_overhang = 1.0F;
  i.rear_overhang = 1.0F;
  i.vehicle_height = 1.0F;

  i.vehicle_length = 1.0F;
  i.vehicle_width = 1.0F;
  i.min_longitudinal_offset = 1.0F;
  i.max_longitudinal_offset = 2.0F;
  i.min_lateral_offset = 1.0F;
  i.max_lateral_offset = 2.0F;
  i.min_height_offset = 1.0F;
  i.max_height_offset = 2.0F;
  return i;
}

StopControlParameter createStopParam()
{
  StopControlParameter i;
  i.stop_margin = 1.0F;
  i.min_behavior_stop_margin = 1.0F;
  i.step_length = 1.0F;
  i.extend_distance = 1.0F;
  i.expand_stop_range = 1.0F;
  i.stop_search_radius = 1.0F;
  return i;
}

SlowDownControlParameter createSlowDownParam()
{
  SlowDownControlParameter i;
  i.slow_down_margin = 1.0F;
  i.expand_slow_down_range = 1.0F;
  i.max_slow_down_vel = 2.0F;
  i.min_slow_down_vel = 1.0F;
  i.max_deceleration = 1.0F;
  i.enable_slow_down = true;
  i.slow_down_search_radius = 1.0F;
  return i;
}

AdaptiveCruiseControlParameter createAccParam()
{
  AdaptiveCruiseControlParameter i;
  i.use_object_to_est_vel = true;
  // add parameter
  return i;
}

// class ObstacleStopPlannerTest : public ::testing::Test
// {
// public:
//   ObstacleStopPlannerTest()
//   {
//     const VehicleInfo vehicle_info = createVehicleParam();

//     const StopControlParameter stop_param = createStopParam();

//     const SlowDownControlParameter slow_down_param = createSlowDownParam();

//     const AdaptiveCruiseControlParameter acc_param = createAccParam();

//     rclcpp::Node dummy_node{"dummy_node"};

//     planner_ = std::make_shared<ObstacleStopPlanner>(
//       &dummy_node,
//       vehicle_info,
//       stop_param,
//       slow_down_param,
//       acc_param);
//   }
//   std::shared_ptr<ObstacleStopPlanner> planner_;
// };

// TEST_F(ObstacleStopPlannerTest, example)
// {
//   EXPECT_EQ(1, 1);
// }
