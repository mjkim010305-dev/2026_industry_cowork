"""/sweep action server (+ the MoveIt-to-Isaac arm bridge when bridge:=true)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('safety_monitor', default_value='false'),
        Node(package='mona_manipulation', executable='moveit_to_isaac_bridge.py',
             name='isaac_arm_bridge', output='screen', condition=IfCondition(LaunchConfiguration('bridge')),
             parameters=[{'use_sim_time': sim, 'relay_joint_states': False,
                          'isaac_state_topic': '/joint_states',
                          'isaac_command_topic': '/joint_commands'}]),
        Node(package='mona_manipulation', executable='sweep_server.py', name='sweep_server',
             output='screen',
             parameters=[{'use_sim_time': sim,
                          'safety_monitor': LaunchConfiguration('safety_monitor')}]),
    ])
