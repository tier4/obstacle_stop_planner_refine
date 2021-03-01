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

#ifndef OBSTACLE_STOP_PLANNER__POLYGON_HPP_
#define OBSTACLE_STOP_PLANNER__POLYGON_HPP_

#include <vector>
#include "boost/assert.hpp"
#include "boost/assign/list_of.hpp"
#include "boost/format.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/linestring.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace motion_planning {

inline static Polygon ConvertToBoostPolygon(std::vector<cv::Point2d>& polygon)
{
  // convert boost polygon
  Polygon boost_polygon;
  for (const auto & point : polygon) {
    boost_polygon.outer().push_back(bg::make<Point>(point.x, point.y));
  }
  boost_polygon.outer().push_back(
    bg::make<Point>(
      polygon.front().x, polygon.front().y));
  return boost_polygon;
}

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__POLYGON_HPP_
