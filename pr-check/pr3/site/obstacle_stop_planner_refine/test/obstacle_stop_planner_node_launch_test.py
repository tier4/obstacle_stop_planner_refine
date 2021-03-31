# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import os
import unittest

from ament_index_python import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
import pytest


@pytest.mark.launch_test
def generate_test_description():
    obstacle_stop_planner_node = Node(
        package='obstacle_stop_planner_refine',
        executable='obstacle_stop_planner_node_exe',
        namespace='test',
        parameters=[
            os.path.join(
                get_package_share_directory('obstacle_stop_planner_refine'),
                'config/obstacle_stop_planner.param.yaml'),
            os.path.join(
                get_package_share_directory('obstacle_stop_planner_refine'),
                'config/adaptive_cruise_control.param.yaml'),
            os.path.join(
                get_package_share_directory('obstacle_stop_planner_refine'),
                'config/test_vehicle_info.param.yaml')
        ])

    context = {'obstacle_stop_planner_node': obstacle_stop_planner_node}

    return LaunchDescription([
        obstacle_stop_planner_node,
        # Start tests right away - no need to wait for anything
        ReadyToTest(),
    ]), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, obstacle_stop_planner_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        assertExitCodes(proc_info, [-15], process=obstacle_stop_planner_node)
