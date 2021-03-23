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

#include "gtest/gtest.h"
#include <memory>
#include <vector>
#include <string>

#include "obstacle_stop_planner/obstacle_stop_planner.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace obstacle_stop_planner;

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

Trajectory convertPointsToTrajectoryWithYaw(
  const std::vector<Point3d> & points)
{
  Trajectory trajectory;
  for (size_t i = 0; i < points.size(); i++) {
    TrajectoryPoint traj_point;
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

void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size)
{
  msg.height = 1U;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.header.frame_id = frame_id;
  // set the fields
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
    4U,
    "x", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1U, sensor_msgs::msg::PointField::FLOAT32);
  // allocate memory so that iterators can be used
  modifier.resize(size);
}

class ObstacleStopPlannerTest : public ::testing::Test
{
public:
  ObstacleStopPlannerTest()
  {
    const VehicleInfo vehicle_info = createVehicleParam();

    const StopControlParameter stop_param = createStopParam();

    const SlowDownControlParameter slow_down_param = createSlowDownParam();

    const AdaptiveCruiseControlParameter acc_param = createAccParam();

    planner_ = std::make_shared<ObstacleStopPlanner>(
      vehicle_info,
      stop_param,
      slow_down_param,
      acc_param);
  }
  std::shared_ptr<ObstacleStopPlanner> planner_;
};

TEST_F(ObstacleStopPlannerTest, pass_through_plan)
{
  // create trajectory
  std::vector<Point3d> points;
  points.emplace_back(0.0, 0.0, 0.0);
  points.emplace_back(1.0, 0.0, 0.0);
  points.emplace_back(2.0, 0.0, 0.0);
  points.emplace_back(3.0, 0.0, 0.0);
  points.emplace_back(4.0, 0.0, 0.0);
  points.emplace_back(5.0, 0.0, 0.0);
  const auto trajectory = convertPointsToTrajectoryWithYaw(points);
  const geometry_msgs::msg::Pose pose = trajectory.points.at(0).pose;
  const auto path = planner_->processTrajectory(trajectory, pose);

  // create transform
  const geometry_msgs::msg::TransformStamped transform;

  // set pointcloud
  const uint32_t max_cloud_size = 10;

  sensor_msgs::msg::PointCloud2 test_msg;
  init_pcl_msg(test_msg, "dummy_frame", max_cloud_size);

  // const sensor_msgs::msg::PointCloud2::SharedPtr pc;
  // *pc = test_msg;
  const auto pc = std::make_shared<sensor_msgs::msg::PointCloud2>(test_msg);
  planner_->updatePointCloud(pc);

  const auto ret = planner_->updatePath(path, pose, transform);

  EXPECT_GT(0, ret.points.size());
}

TEST_F(ObstacleStopPlannerTest, slow_down_plan)
{
  EXPECT_EQ(1, 1);
}

// TEST(ObstacleStopPlannerTest, stop_plan)
// {

// }

// TEST(ObstacleStopPlannerTest, acc_plan)
// {

// }
