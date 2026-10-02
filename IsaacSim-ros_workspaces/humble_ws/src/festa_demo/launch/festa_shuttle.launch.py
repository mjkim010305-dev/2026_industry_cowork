"""festa_demo shuttle (2026-10-02, user): pick the part in front, drive to the far goal,
put the part down in front there, pick it up again, drive back to the start, put it down,
pick it up ... forever. Green boxes on the way are handled as in festa_demo.launch.py.

festa_demo.launch.py with pick:=true pick_first:=true shuttle:=true round_trips:=0; every
other festa_demo.launch.py argument passes through, e.g.
    ros2 launch festa_demo festa_shuttle.launch.py mode:=real
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='sim', description="'sim' (Isaac) or 'real' (robot)"),
        DeclareLaunchArgument('round_trips', default_value='0', description='0 = forever'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('festa_demo'), 'launch', 'festa_demo.launch.py')),
            launch_arguments={'mode': LaunchConfiguration('mode'), 'pick': 'true', 'pick_first': 'true',
                              'shuttle': 'true',
                              'round_trips': LaunchConfiguration('round_trips')}.items()),
    ])
