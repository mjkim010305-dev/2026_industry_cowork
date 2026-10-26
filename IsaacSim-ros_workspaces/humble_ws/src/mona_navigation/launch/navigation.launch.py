import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map yaml'),
        DeclareLaunchArgument('params_file', description='Full path to the Nav2 params yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])
