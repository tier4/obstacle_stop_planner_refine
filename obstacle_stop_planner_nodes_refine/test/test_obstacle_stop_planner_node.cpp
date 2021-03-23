// Copyright 2020 The Autoware Foundation
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

#include <obstacle_stop_planner_nodes/obstacle_stop_planner_node.hpp>

#include <memory>
#include <chrono>

#include "gtest/gtest.h"

using sensor_msgs::msg::PointCloud2;
using autoware_planning_msgs::msg::Trajectory;

class ObstacleStopPlannerNodeTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_FALSE(rclcpp::ok());
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    fake_node_ = std::make_shared<rclcpp::Node>("fake_node");

    using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
    dummy_pointcloud_pub_ =
      fake_node_->create_publisher<PointCloud2>(
      "input/pointcloud", rclcpp::QoS{1}, PubAllocT{});
    dummy_path_pub_ =
      fake_node_->create_publisher<Trajectory>(
      "input/trajectory", rclcpp::QoS{1}, PubAllocT{});
    dummy_velocity_pub_ =
      fake_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "input/twist", rclcpp::QoS{1}, PubAllocT{});
    dummy_object_pub_ =
      fake_node_->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>(
      "input/objects", rclcpp::QoS{1}, PubAllocT{});
    path_sub_ =
      fake_node_->create_subscription<Trajectory>(
      "output/trajectory", rclcpp::QoS{1},
      [this](const Trajectory::SharedPtr msg)
      {this->path_msg_ = *msg;});

    rclcpp::NodeOptions node_options{};

    planner_ = std::make_shared<obstacle_stop_planner_nodes::ObstacleStopPlannerNode>(node_options);
  }

  std::shared_ptr<obstacle_stop_planner_nodes::ObstacleStopPlannerNode> planner_;
  rclcpp::Node::SharedPtr fake_node_{nullptr};
  rclcpp::Publisher<PointCloud2>::SharedPtr dummy_pointcloud_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr dummy_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dummy_velocity_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DynamicObjectArray>::SharedPtr dummy_object_pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr path_sub_;
  boost::optional<Trajectory> path_msg_;
};

TEST_F(ObstacleStopPlannerNodeTest, plan_simple_trajectory)
{
  using namespace std::chrono_literals;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fake_node_);
  executor.add_node(planner_);

  // TODO(KS): Publish data


  using namespace std::chrono_literals;
  rclcpp::WallRate rate(100ms);
  while (rclcpp::ok()) {
    executor.spin_some();
    if (path_msg_.has_value()) {
      break;
    }
    rate.sleep();
  }

  // Check data
  ASSERT_EQ(0, 1);
}
