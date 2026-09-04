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

"""Launch the RGB-D obstacle localizer (depth + TF -> obstacle pose in map)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    # Force a real bool so `use_sim_time:=true` is not passed as the string "true".
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_rect_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="custom_nav2_bt_plugins",
            executable="rgbd_obstacle_localizer",
            name="rgbd_obstacle_localizer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "gate_topic": "/obstacle/is_movable",
                "obstacle_pose_topic": "/movable_obstacle/pose",
                "target_frame": "map",
                "publish_rate": 10.0,
                "data_timeout": 1.0,
                "gate_timeout": 2.0,
                "transform_timeout": 0.2,
                "localizer_plugin": "custom_nav2_bt_plugins::DepthCentroidLocalizer",
                "localizer_name": "depth_localizer",
                # DepthCentroidLocalizer parameters (namespaced by localizer_name):
                "depth_localizer.roi_width_ratio": 0.4,
                "depth_localizer.roi_height_ratio": 0.6,
                "depth_localizer.min_range": 0.2,
                "depth_localizer.max_range": 6.0,
                "depth_localizer.near_percentile": 0.15,
                "depth_localizer.depth_band": 0.25,
                "depth_localizer.min_points": 50,
            }],
        ),
    ])
