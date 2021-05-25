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
#include "obstacle_stop_planner/point_helper.hpp"


TEST(getBackwardPointFromBasePoint, returnValue) {
  Point2d line1{1.0, 1.0};
  Point2d line2{1.0, -1.0};
  Point2d base_point{0.0, 0.0};
  double backward_length = 1.0;
  auto ret = obstacle_stop_planner::PointHelper::getBackwardPointFromBasePoint(
    line1, line2, base_point, backward_length);

  Point2d expect = Point2d{0.0, -1.0};
  EXPECT_EQ(expect, ret);
}

TEST(getNearestPoint, returnValue) {
  geometry_msgs::msg::Pose base_pose;
  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::PointXYZ point;
  point.x = 10.0;
  point.y = 10.0;
  point.z = 10.0;
  pc.points.push_back(point);
  point.x = 20.0;
  point.y = 20.0;
  point.z = 20.0;
  pc.points.push_back(point);
  auto ret = obstacle_stop_planner::PointHelper::getNearestPoint(pc, base_pose);
  EXPECT_EQ(10.0, ret.point.x());
  EXPECT_EQ(10.0, ret.point.y());
}
