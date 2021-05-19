// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef OBSTACLE_STOP_PLANNER__UTIL__PARAMETER_HELPER_HPP_
#define OBSTACLE_STOP_PLANNER__UTIL__PARAMETER_HELPER_HPP_

#include <string>
#include <vector>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

namespace
{
template<typename T>
T declare_parameter(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  return node->declare_parameter(name, rclcpp::ParameterValue(default_value)).get<T>();
}

template<typename T>
void update_parameter(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}
}  // namespace

#endif  // OBSTACLE_STOP_PLANNER__UTIL__PARAMETER_HELPER_HPP_
