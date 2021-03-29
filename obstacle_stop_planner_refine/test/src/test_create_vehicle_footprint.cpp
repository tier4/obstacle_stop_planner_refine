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
#include "obstacle_stop_planner/util/create_vehicle_footprint.hpp"


TEST(createVehicleFootprint, returnValue) {
  VehicleInfo vehicle_info;
  vehicle_info.front_overhang = 0.5;
  vehicle_info.rear_overhang = 0.3;
  vehicle_info.wheel_base = 2.0;
  vehicle_info.wheel_tread = 0.1;
  vehicle_info.left_overhang = 0.2;
  vehicle_info.right_overhang = 0.2;

  const auto ring2d = obstacle_stop_planner::createVehicleFootprint(vehicle_info);
  EXPECT_EQ(ring2d.size(), 5U);
  EXPECT_EQ(ring2d.at(0).x(), 2.5);
  EXPECT_EQ(ring2d.at(0).y(), 0.25);
}
