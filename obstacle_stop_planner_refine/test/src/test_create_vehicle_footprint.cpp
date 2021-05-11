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
  const auto vehicle_info =
    vehicle_info_util::createVehicleInfo(0.0, 0.0, 2.0, 0.1, 0.5, 0.3, 0.2, 0.2, 0.0);

  const auto ring2d = obstacle_stop_planner::createVehicleFootprint(vehicle_info);
  EXPECT_EQ(ring2d.size(), 5U);
  EXPECT_EQ(ring2d.at(0).x(), 2.5);
  EXPECT_EQ(ring2d.at(0).y(), 0.25);
}
