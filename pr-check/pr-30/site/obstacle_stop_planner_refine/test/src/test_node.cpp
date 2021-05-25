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

#include <obstacle_stop_planner/node.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "tf2_ros/transform_broadcaster.h"

using sensor_msgs::msg::PointCloud2;
using autoware_planning_msgs::msg::Trajectory;
using namespace std::chrono_literals;

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
class ObstacleStopPlannerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    fake_node_ = std::make_shared<rclcpp::Node>("fake_node");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(fake_node_);

    using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
    dummy_pointcloud_pub_ =
      fake_node_->create_publisher<PointCloud2>(
      "/obstacle_stop_planner/input/pointcloud", rclcpp::QoS{1}, PubAllocT{});
    dummy_path_pub_ =
      fake_node_->create_publisher<Trajectory>(
      "/obstacle_stop_planner/input/trajectory", rclcpp::QoS{1}, PubAllocT{});
    dummy_velocity_pub_ =
      fake_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/obstacle_stop_planner/input/twist", rclcpp::QoS{1}, PubAllocT{});
    dummy_object_pub_ =
      fake_node_->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
      "/obstacle_stop_planner/input/objects", rclcpp::QoS{1}, PubAllocT{});
    path_sub_ =
      fake_node_->create_subscription<Trajectory>(
      "/obstacle_stop_planner/output/trajectory", rclcpp::QoS{1},
      [this](const Trajectory::SharedPtr msg)
      {this->path_msg_ = *msg;});

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

    planner_ = std::make_shared<obstacle_stop_planner::ObstacleStopPlannerNode>(node_options);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<obstacle_stop_planner::ObstacleStopPlannerNode> planner_;
  rclcpp::Node::SharedPtr fake_node_{nullptr};
  rclcpp::Publisher<PointCloud2>::SharedPtr dummy_pointcloud_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr dummy_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dummy_velocity_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr dummy_object_pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr path_sub_;
  boost::optional<Trajectory> path_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(ObstacleStopPlannerNodeTest, plan_simple_trajectory)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fake_node_);
  executor.add_node(planner_);
  executor.spin_some();

  auto current_time = fake_node_->now();

  // create /tf
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.header.stamp = current_time;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  // publish point cloud
  const uint32_t max_cloud_size = 10;

  sensor_msgs::msg::PointCloud2 test_msg;
  init_pcl_msg(test_msg, "base_link", max_cloud_size);
  test_msg.header.stamp = current_time;
  dummy_pointcloud_pub_->publish(test_msg);

  // current velocity
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.twist.angular.x = 0.0F;
  dummy_velocity_pub_->publish(twist_msg);

  executor.spin_some();

  // create trajectory
  const std::vector<Point3d> points{
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0},
    {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0},
    {4.0, 0.0, 0.0},
    {5.0, 0.0, 0.0}
  };

  auto trajectory = convertPointsToTrajectoryWithYaw(points);
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = current_time;
  dummy_path_pub_->publish(trajectory);

  rclcpp::WallRate rate(100ms);
  while (rclcpp::ok()) {
    current_time = fake_node_->now();
    tf.header.stamp = current_time;
    tf_broadcaster_->sendTransform(tf);
    test_msg.header.stamp = current_time;
    dummy_pointcloud_pub_->publish(test_msg);
    trajectory.header.stamp = current_time;
    dummy_path_pub_->publish(trajectory);

    executor.spin_some();
    if (path_msg_.has_value()) {
      break;
    }
    rate.sleep();
  }

  // Check data
  for (size_t i = 0; i < path_msg_->points.size(); i++) {
    ASSERT_EQ(trajectory.points[i], path_msg_->points[i]);
  }
}