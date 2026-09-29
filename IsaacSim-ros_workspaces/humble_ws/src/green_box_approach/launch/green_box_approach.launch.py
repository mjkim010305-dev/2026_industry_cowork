#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the green box detector node.

Deliberately does not set any tuning parameter here (HSV range, min_area,
box dimensions, bearing margin, ...) - those live only as node defaults in
``detector_node.py``, with ``config/green_box_approach_params.yaml`` as an
optional override file. A launch file that copies tuning values bit this
project before (a stale copy silently outlived the node default it was
supposed to track) - see the Session 11 contract.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    # No params_file given -> run purely on the node's own declared defaults,
    # rather than pointing at a yaml (hardcoded or not) by default.
    parameters = [params_file] if params_file else []
    return [Node(
        package='green_box_approach',
        executable='detector_node',
        name='green_box_detector',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Optional YAML file overriding node parameter defaults '
                     '(see config/green_box_approach_params.yaml for an example).',
    )

    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=_launch_setup),
    ])
