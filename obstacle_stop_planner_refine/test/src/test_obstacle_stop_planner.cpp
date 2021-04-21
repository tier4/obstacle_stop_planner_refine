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

// FIXME: Rename function name
Trajectory convertPointsToTrajectoryWithYaw2(
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

class ObstacleStopPlannerTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    rclcpp::NodeOptions node_options{};

    node_options.append_parameter_override("ready_vehicle_info_param", true);
    node_options.append_parameter_override("wheel_radius", 0.39F);
    node_options.append_parameter_override("wheel_width", 0.42F);
    node_options.append_parameter_override("wheel_base", 2.74F);
    node_options.append_parameter_override("wheel_tread", 1.63F);
    node_options.append_parameter_override("front_overhang", 1.0F);
    node_options.append_parameter_override("rear_overhang", 1.03F);
    node_options.append_parameter_override("left_overhang", 0.1F);
    node_options.append_parameter_override("right_overhang", 0.1F);
    node_options.append_parameter_override("vehicle_height", 2.5F);

    node_options.append_parameter_override(
      "adaptive_cruise_control.use_object_to_estimate_vel", false);

    fake_node_ = std::make_shared<rclcpp::Node>("fake_node", node_options);

    const auto vehicle_info = std::make_shared<vehicle_info_util::VehicleInfo>(vehicle_info_util::VehicleInfo::create(*fake_node_));

    stop_param_ = std::make_shared<obstacle_stop_planner::StopControlParameter>();
    slow_down_param_ = std::make_shared<obstacle_stop_planner::SlowDownControlParameter>();
    acc_param_ = std::make_shared<obstacle_stop_planner::AdaptiveCruiseControlParameter>();

    planner_ = std::make_shared<obstacle_stop_planner::ObstacleStopPlanner>(
      fake_node_->get_node_logging_interface(),
      fake_node_->get_node_clock_interface(),
      vehicle_info, stop_param_, slow_down_param_, acc_param_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr fake_node_;
  std::shared_ptr<obstacle_stop_planner::ObstacleStopPlanner> planner_;
  std::shared_ptr<obstacle_stop_planner::StopControlParameter> stop_param_;
  std::shared_ptr<obstacle_stop_planner::SlowDownControlParameter> slow_down_param_;
  std::shared_ptr<obstacle_stop_planner::AdaptiveCruiseControlParameter> acc_param_;
};

TEST_F(ObstacleStopPlannerTest, searchCandidateObstacle)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
    {30.0, 0.0, 0.0},
    {40.0, 0.0, 0.0},
    {50.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  std::vector<Point3d> obstacles{
    {30.1, 0.0, 0.0},
    {30.2, 0.0, 0.0},
    {30.3, 0.0, 0.0},
  };

  const auto candidate_obstacles = planner_->searchCandidateObstacle(trajectory, obstacles);

  ASSERT_EQ(candidate_obstacles.size(), 3U);
}

TEST_F(ObstacleStopPlannerTest, findCollisionPoint_nothing)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
    {30.0, 0.0, 0.0},
    {40.0, 0.0, 0.0},
    {50.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  std::vector<Point3d> obstacles{
    {30.1, 50.0, 0.0},
    {30.2, 50.0, 0.0},
    {30.3, 50.0, 0.0},
  };

  const auto candidate_obstacles = planner_->findCollisionPoint(trajectory, obstacles);

  ASSERT_FALSE(candidate_obstacles.has_value());
}

TEST_F(ObstacleStopPlannerTest, findCollisionPoint_has_value)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
    {30.0, 0.0, 0.0},
    {40.0, 0.0, 0.0},
    {50.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  std::vector<Point3d> obstacles{
    {30.1, 0.0, 0.0},
    {30.2, 0.0, 0.0},
    {30.3, 0.0, 0.0},
  };

  const auto candidate_obstacles = planner_->findCollisionPoint(trajectory, obstacles);

  ASSERT_TRUE(candidate_obstacles.has_value());
  ASSERT_EQ(candidate_obstacles.value().obstacle_point, obstacles.at(0).to_2d());
}

TEST_F(ObstacleStopPlannerTest, planSlowDown)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
    {30.0, 0.0, 0.0},
    {40.0, 0.0, 0.0},
    {50.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  // Add velocity
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points.at(i).twist.linear.x = 5.0;
  }

  // Set parameter
  slow_down_param_->enable_slow_down = true;
  slow_down_param_->min_slow_down_vel = 2.0;
  slow_down_param_->max_slow_down_vel = 3.0;
  slow_down_param_->max_deceleration = 1.0;
  slow_down_param_->expand_slow_down_range = 1.0;

  planner_->updateParameters(stop_param_, slow_down_param_, acc_param_);

  obstacle_stop_planner::Collision collision;
  collision.segment_index = 3;
  collision.obstacle_point = Point2d{35.0, 0.0};

  std::vector<Point3d> obstacles {
    collision.obstacle_point.to_3d(),
  };

  const auto processed_trajectory = planner_->planSlowDown(trajectory, collision, obstacles);

  // Get expected velocity
  const auto lateral_deviation = autoware_utils::calcLateralDeviation(trajectory.points.at(3).pose, autoware_utils::toMsg(collision.obstacle_point.to_3d()));

  const auto target_velocity = planner_->calcSlowDownTargetVel(lateral_deviation);

  ASSERT_EQ(processed_trajectory.points.at(0).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(1).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(2).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(3).twist.linear.x, target_velocity);
  ASSERT_EQ(processed_trajectory.points.at(4).twist.linear.x, target_velocity);
  ASSERT_EQ(processed_trajectory.points.at(5).twist.linear.x, 5.0);
}

TEST_F(ObstacleStopPlannerTest, planObstacleStop)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
    {30.0, 0.0, 0.0},
    {40.0, 0.0, 0.0},
    {50.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  // Add velocity
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points.at(i).twist.linear.x = 5.0;
  }

  obstacle_stop_planner::Collision collision;
  collision.segment_index = 3;
  collision.obstacle_point = Point2d{35.0, 0.0};

  const auto processed_trajectory = planner_->planObstacleStop(trajectory, collision);

  ASSERT_EQ(processed_trajectory.points.at(0).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(1).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(2).twist.linear.x, 5.0);
  ASSERT_EQ(processed_trajectory.points.at(3).twist.linear.x, 0.0);
  ASSERT_EQ(processed_trajectory.points.at(4).twist.linear.x, 0.0);
  ASSERT_EQ(processed_trajectory.points.at(5).twist.linear.x, 0.0);
}

TEST_F(ObstacleStopPlannerTest, findFrontObstacles_found)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  std::vector<Point3d> obstacles {
    {15.0, 0.0, 0.0}
  };

  planner_->search_radius_ = 5.0;

  const auto ret = planner_->findFrontObstacles(trajectory.points.at(1), obstacles);

  ASSERT_TRUE(ret);
}

TEST_F(ObstacleStopPlannerTest, findFrontObstacles_not_found)
{
  std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {10.0, 0.0, 0.0},
    {20.0, 0.0, 0.0},
  };
  auto trajectory = convertPointsToTrajectoryWithYaw2(points);

  std::vector<Point3d> obstacles {
    {6.0, 0.0, 0.0}
  };

  planner_->search_radius_ = 5.0;

  const auto ret = planner_->findFrontObstacles(trajectory.points.at(1), obstacles);

  ASSERT_FALSE(ret);
}
