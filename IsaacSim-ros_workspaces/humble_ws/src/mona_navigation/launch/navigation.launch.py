import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = LaunchConfiguration('params_file').perform(context)
    judge_on_path = LaunchConfiguration('judge_on_path').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')

    actions = []
    if judge_on_path:
        # RewrittenYaml only rewrites keys that already exist in the yaml, and the stock
        # params must stay without default_nav_to_pose_bt_xml, so the key is set here.
        bt_xml = os.path.join(
            get_package_share_directory('mona_navigation'), 'bt', 'navigate_judge_on_path.xml')
        with open(params_file, 'r') as f:
            data = yaml.safe_load(f)
        data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
        with tempfile.NamedTemporaryFile(
                mode='w', suffix='.yaml', prefix='mona_judge_on_path_', delete=False) as f:
            yaml.dump(data, f)
            params_file = f.name
        actions.append(Node(
            package='mona_navigation',
            executable='path_obstacle_monitor.py',
            name='path_obstacle_monitor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map yaml'),
        DeclareLaunchArgument('params_file', description='Full path to the Nav2 params yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('judge_on_path', default_value='false',
                              description='true: judge-on-path BT + path_obstacle_monitor'),
        OpaqueFunction(function=_launch_setup),
    ])
