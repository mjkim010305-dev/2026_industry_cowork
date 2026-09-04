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

"""Launch the RGB-D obstacle monitor that feeds the movable-obstacle BT plugins."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/depth",
                              description="RGB-D depth image topic (16UC1/mono16 mm or 32FC1 m)"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="custom_nav2_bt_plugins",
            executable="rgbd_obstacle_monitor",
            name="rgbd_obstacle_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "depth_topic": depth_topic,
                "camera_info_topic": "",
                "movable_obstacle_topic": "/movable_obstacle/detected",
                "obstacle_present_topic": "/obstacle/present",
                "publish_rate": 10.0,
                "depth_timeout": 1.0,
                "detector_plugin": "custom_nav2_bt_plugins::DepthThresholdDetector",
                "detector_name": "depth_detector",
                # DepthThresholdDetector parameters (namespaced by detector_name):
                "depth_detector.roi_width_ratio": 0.4,
                "depth_detector.roi_height_ratio": 0.6,
                "depth_detector.stop_distance": 0.8,
                "depth_detector.min_range": 0.15,
                "depth_detector.min_fill_ratio": 0.06,
                "depth_detector.movable_confirm_time": 1.5,
                "depth_detector.clear_time": 1.0,
            }],
        ),
    ])
