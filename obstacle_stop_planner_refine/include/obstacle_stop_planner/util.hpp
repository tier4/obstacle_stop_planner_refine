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
#ifndef OBSTACLE_STOP_PLANNER__UTIL_HPP_
#define OBSTACLE_STOP_PLANNER__UTIL_HPP_

#include "opencv2/core/core.hpp"

namespace motion_planning
{
  cv::Point2d calcCentroid(const std::vector<cv::Point2d> pointcloud)
  {
    cv::Point2d centroid;
    centroid.x = 0;
    centroid.y = 0;
    for (const auto &point : pointcloud)
    {
      centroid.x += point.x;
      centroid.y += point.y;
    }
    centroid.x = centroid.x / static_cast<double>(pointcloud.size());
    centroid.y = centroid.y / static_cast<double>(pointcloud.size());
    return centroid;
  }
}

#endif // OBSTACLE_STOP_PLANNER__UTIL_HPP_
