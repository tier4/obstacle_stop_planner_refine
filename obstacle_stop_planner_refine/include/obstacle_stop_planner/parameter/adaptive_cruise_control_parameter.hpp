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

#ifndef OBSTACLE_STOP_PLANNER__PARAMETER__ADAPTIVE_CRUISE_CONTROL_PARAMETER_HPP_
#define OBSTACLE_STOP_PLANNER__PARAMETER__ADAPTIVE_CRUISE_CONTROL_PARAMETER_HPP_

#include <obstacle_stop_planner/visibility_control.hpp>

namespace obstacle_stop_planner
{
struct OBSTACLE_STOP_PLANNER_PUBLIC AdaptiveCruiseControlParameter
{
  //!< @brief use tracking objects for estimating object velocity or not
  bool use_object_to_est_vel;

  //!< @brief use pcl for estimating object velocity or not
  bool use_pcl_to_est_vel;

  //!< @brief consider forward vehicle velocity to self upper velocity or not
  bool consider_obj_velocity;

  //!< @brief The distance to extend the polygon length the object in pointcloud-object matching
  double object_polygon_length_margin;

  //!< @brief The distance to extend the polygon width the object in pointcloud-object matching
  double object_polygon_width_margin;

  //!< @breif Maximum time difference treated as continuous points in speed estimation using a
  // point cloud
  double valid_est_vel_diff_time;

  //!< @brief Time width of information used for speed estimation in speed estimation using a
  // point cloud
  double valid_vel_que_time;

  //!< @brief Maximum value of valid speed estimation results in speed estimation using a point
  // cloud
  double valid_est_vel_max;

  //!< @brief Minimum value of valid speed estimation results in speed estimation using a point
  // cloud
  double valid_est_vel_min;

  //!< @brief Embed a stop line if the maximum speed calculated by ACC is lower than this speed
  double thresh_vel_to_stop;

  /* parameter for acc */
  //!< @brief threshold of forward obstacle velocity to insert stop line (to stop acc)
  double obstacle_stop_velocity_thresh;

  //!< @brief supposed minimum acceleration in emergency stop
  double emergency_stop_acceleration;

  //!< @brief supposed minimum acceleration of forward vehicle in emergency stop
  double obstacle_emergency_stop_acceleration;

  //!< @brief supposed idling time to start emergency stop
  double emergency_stop_idling_time;

  //!< @brief minimum distance of emergency stop
  double min_dist_stop;

  //!< @brief supposed maximum acceleration in active cruise control
  double max_standard_acceleration;

  //!< @brief supposed minimum acceleration(deceleration) in active cruise control
  double min_standard_acceleration;

  //!< @brief supposed idling time to react object in active cruise control
  double standard_idling_time;

  //!< @brief minimum distance in active cruise control
  double min_dist_standard;

  //!< @brief supposed minimum acceleration of forward obstacle
  double obstacle_min_standard_acceleration;

  //!< @brief margin to insert upper velocity
  double margin_rate_to_change_vel;

  //!< @brief use time-compensation to calculate distance to forward vehicle
  bool use_time_compensation_to_dist;

  //!< @brief gain of lowpass filter of upper velocity
  double lowpass_gain_;

  //!< @brief when failed to estimate velocity, use rough velocity estimation or not
  bool use_rough_est_vel;

  //!< @brief in rough velocity estimation, front car velocity is
  //!< estimated as self current velocity * this value
  double rough_velocity_rate;

  /* parameter for pid used in acc */
  //!< @brief coefficient P in PID control (used when target dist -current_dist >=0)
  double p_coeff_pos;

  //!< @brief coefficient P in PID control (used when target dist -current_dist <0)
  double p_coeff_neg;

  //!< @brief coefficient D in PID control (used when delta_dist >=0)
  double d_coeff_pos;

  //!< @brief coefficient D in PID control (used when delta_dist <0)
  double d_coeff_neg;

  double d_coeff_valid_time = 1.0;
  double d_coeff_valid_diff_vel = 20.0;
  double d_max_vel_norm = 3.0;
};
}  // namespace obstacle_stop_planner

#endif  // OBSTACLE_STOP_PLANNER__PARAMETER__ADAPTIVE_CRUISE_CONTROL_PARAMETER_HPP_
