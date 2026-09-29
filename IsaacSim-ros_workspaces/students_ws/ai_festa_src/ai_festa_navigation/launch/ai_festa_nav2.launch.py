"""AI Festa navigation, using Isaac TF and the standard Humble velocity chain."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    own = get_package_share_directory('ai_festa_navigation')
    nav = get_package_share_directory('nav2_bringup')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': '', 'use_namespace': 'False', 'slam': 'False',
            'map': '/home/students/workspace/src/ai-festa-map.yaml',
            'params_file': os.path.join(own, 'config', 'ai_festa_nav2_params.yaml'),
            'use_sim_time': 'true', 'autostart': 'true',
            'use_composition': 'False', 'use_respawn': 'False',
        }.items())])
