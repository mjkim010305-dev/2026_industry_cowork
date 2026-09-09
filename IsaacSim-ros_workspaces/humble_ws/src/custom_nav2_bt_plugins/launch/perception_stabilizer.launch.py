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

"""Launch the Perception Stabilizer (VLM detect + depth -> stabilised estimate).

Replaces rgbd_obstacle_localizer on the pick-and-navigate (manipulator) path:
publishes /perception_state, /obstacle_pose_stable, /obstacle_pose_confidence
for the IsObstacleTracking / LockApproachPose BT nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    detect_topic = LaunchConfiguration("detect_topic")
    roi_topic = LaunchConfiguration("roi_topic")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("depth_topic", default_value="/depth_camera"),
        DeclareLaunchArgument("camera_info_topic", default_value="/depth_camera_info"),
        DeclareLaunchArgument("detect_topic", default_value="/obstacle/is_movable"),
        # Empty -> centred ratio crop. Set to the VLM's sensor_msgs/RegionOfInterest
        # topic once it exists; nothing else changes.
        DeclareLaunchArgument("roi_topic", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="custom_nav2_bt_plugins",
            executable="perception_stabilizer.py",
            name="perception_stabilizer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "detect_topic": detect_topic,
                "roi_topic": roi_topic,
                "target_frame": "map",
                "process_rate": 10.0,
                "buffer_duration": 1.0,
                "voxel_size": 0.02,
                "min_points": 300,
                "roi_width_ratio": 0.4,
                "roi_height_ratio": 0.6,
                "min_range": 0.2,
                "max_range": 6.0,
                # state machine
                "state_window": 1.0,
                "tracking_enter_rate": 0.7,
                "converge_std_deg": 5.0,
                "tracking_lost_timeout": 0.7,
                "candidate_timeout": 3.0,
                "detect_timeout": 2.0,
            }],
        ),
    ])
