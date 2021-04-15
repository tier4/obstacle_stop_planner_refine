// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <obstacle_stop_planner/obstacle_stop_planner.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

using sensor_msgs::msg::PointCloud2;
using autoware_planning_msgs::msg::Trajectory;
using namespace std::chrono_literals;

class ObstacleStopPlannerTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    fake_node_ = std::make_shared<rclcpp::Node>("fake_node");
    obstacle_stop_planner::StopControlParameter stop_param;
    obstacle_stop_planner::SlowDownControlParameter slow_down_param;
    obstacle_stop_planner::AdaptiveCruiseControlParameter acc_param;

    // planner_ = std::make_shared<obstacle_stop_planner::ObstacleStopPlanner>(fake_node_, stop_param, slow_down_param, acc_param);
  }

  rclcpp::Node::SharedPtr fake_node_;
  std::shared_ptr<obstacle_stop_planner::ObstacleStopPlanner> planner_;
};

// TEST_F(ObstacleStopPlannerTest, searchCandidateObstacle)
// {

// }
