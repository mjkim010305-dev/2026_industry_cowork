"""AI Festa L-course scenario in one launch.

    ros2 launch festa_bringup festa_scenario.launch.py mode:=sim    # Isaac already playing
    ros2 launch festa_bringup festa_scenario.launch.py mode:=real   # robot bringup already up

Starts, in order:
  1. (sim only) moveit_to_isaac_bridge.py - /arm_controller and /gripper_controller
     action servers on top of Isaac's /joint_commands (on the real robot the
     ros2_control controllers from hardware.launch.py provide them).
  2. (sim only, publish_camera_tf) base_link -> camera_optical static TF.
  3. Nav2 bringup (map + params, BT = bt/festa_l_course.xml).
  4. green_box_approach detector (HSV + lidar) -> green_box/pose, green_box/detected.
  5. festa_action/sweep_action_server.py -> /sweep (safety cutoff on for real,
     off for sim: Isaac has no joint current).
  6. After goal_delay s: send_goal - initial pose, then one NavigateToPose goal.

The robot bringup itself (Isaac, or hardware.launch.py + lidar + camera on the
robot) is NOT part of this launch.

sweep_action_server.py and moveit_to_isaac_bridge.py are plain scripts in the
workspace source tree, not installed; they are found under ws_src (default:
the src/ next to this package's install/).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

PKG = 'festa_bringup'

# Per-mode defaults; any of them can be overridden on the command line.
MODE_DEFAULTS = {
    'sim': {
        'map': 'l_course_sim.yaml',
        'image_topic': '/front_camera',
        'camera_info_topic': '/front_camera_info',
        'camera_frame': 'camera_optical',
        'publish_camera_tf': 'true',
        'safety_monitor': 'false',
        'isaac_bridge': 'true',
        'goal_x': '1.75', 'goal_y': '-1.75', 'goal_yaw': '-1.5708',
    },
    'real': {
        'map': 'l_course_real.yaml',
        'image_topic': '/camera/camera/color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'camera_frame': 'camera_color_optical_frame',
        'publish_camera_tf': 'false',
        'safety_monitor': 'true',
        'isaac_bridge': 'false',
        'goal_x': '0.0', 'goal_y': '0.0', 'goal_yaw': '0.0',
    },
}
ARGS = ['map', 'image_topic', 'camera_info_topic', 'camera_frame', 'publish_camera_tf',
        'safety_monitor', 'isaac_bridge', 'goal_x', 'goal_y', 'goal_yaw']


def _launch(context):
    share = get_package_share_directory(PKG)
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODE_DEFAULTS:
        raise RuntimeError(f"mode must be 'sim' or 'real', got '{mode}'")
    cfg = {k: LaunchConfiguration(k).perform(context) or MODE_DEFAULTS[mode][k] for k in ARGS}
    sim = mode == 'sim'
    use_sim_time = 'true' if sim else 'false'
    map_yaml = cfg['map'] if os.path.isabs(cfg['map']) else os.path.join(share, 'maps', cfg['map'])
    bt_xml = LaunchConfiguration('bt_xml').perform(context) or os.path.join(share, 'bt', 'festa_l_course.xml')
    params = LaunchConfiguration('params_file').perform(context) or os.path.join(share, 'params', 'l_course_nav2.yaml')
    # install/festa_bringup/share/festa_bringup -> <ws>/src
    ws_src = LaunchConfiguration('ws_src').perform(context) or os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share)))), 'src')
    sweep_server = os.path.join(ws_src, 'festa_manipulation', 'festa_action', 'sweep_action_server.py')
    bridge = os.path.join(ws_src, 'moveit_to_isaac_bridge.py')
    for path in [map_yaml, bt_xml, params, sweep_server] + ([bridge] if cfg['isaac_bridge'] == 'true' else []):
        if not os.path.isfile(path):
            raise RuntimeError(f'festa_scenario: missing {path}')

    actions = []
    if cfg['isaac_bridge'] == 'true':
        actions.append(ExecuteProcess(
            name='isaac_arm_bridge', output='screen',
            cmd=['python3', bridge, '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', 'relay_joint_states:=false', '-p', 'isaac_state_topic:=/joint_states',
                 '-p', 'isaac_command_topic:=/joint_commands']))
    if cfg['publish_camera_tf'] == 'true':
        # Measured base_link -> optical transform of the simulated TB3 camera
        # (same values as run_green.sh / vlm_perception.launch.py).
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf',
            arguments=['--x', '0.017', '--y', '0.0', '--z', '0.4609',
                       '--qx', '-0.524453', '--qy', '0.547077', '--qz', '-0.460846', '--qw', '0.461819',
                       '--frame-id', 'base_link', '--child-frame-id', cfg['camera_frame']],
            parameters=[{'use_sim_time': sim}]))

    nav2_params = RewrittenYaml(source_file=params, param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
                                convert_types=True)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml, 'params_file': nav2_params, 'use_sim_time': use_sim_time,
                          'autostart': 'true'}.items()))

    actions.append(Node(
        package='green_box_approach', executable='detector_node', name='green_box_detector',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'image_topic': cfg['image_topic'],
            'camera_info_topic': cfg['camera_info_topic'],
            'scan_topic': '/scan',
            'camera_frame': cfg['camera_frame'],
            'output_frame': 'map',
            'box_depth_m': float(LaunchConfiguration('box_depth').perform(context)),
            'box_height_m': float(LaunchConfiguration('box_height').perform(context)),
        }]))

    actions.append(ExecuteProcess(
        name='sweep_action_server', output='screen',
        cmd=['python3', sweep_server, '--ros-args', '-p', f"safety_monitor:={cfg['safety_monitor']}"]))

    actions.append(TimerAction(
        period=float(LaunchConfiguration('goal_delay').perform(context)),
        actions=[Node(
            package=PKG, executable='send_goal', name='festa_send_goal', output='screen',
            parameters=[{
                'use_sim_time': sim,
                'goal_x': float(cfg['goal_x']), 'goal_y': float(cfg['goal_y']),
                'goal_yaw': float(cfg['goal_yaw']),
                'set_initial_pose': LaunchConfiguration('set_initial_pose').perform(context) == 'true',
                'initial_x': float(LaunchConfiguration('initial_x').perform(context)),
                'initial_y': float(LaunchConfiguration('initial_y').perform(context)),
                'initial_yaw': float(LaunchConfiguration('initial_yaw').perform(context)),
            }])]))
    return actions


def generate_launch_description():
    decl = [
        DeclareLaunchArgument('mode', default_value='sim', description="'sim' (Isaac) or 'real' (robot)"),
        DeclareLaunchArgument('bt_xml', default_value='', description='BT xml (default: bt/festa_l_course.xml)'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 params (default: params/l_course_nav2.yaml)'),
        DeclareLaunchArgument('ws_src', default_value='',
                              description='workspace src/ holding festa_manipulation and moveit_to_isaac_bridge.py'),
        DeclareLaunchArgument('box_depth', default_value='0.185', description='green box depth [m] (detector)'),
        DeclareLaunchArgument('box_height', default_value='0.12', description='green box height [m] (detector)'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_delay', default_value='20.0',
                              description='seconds before the initial pose / goal are sent'),
    ] + [DeclareLaunchArgument(k, default_value='', description=f'per-mode default: {MODE_DEFAULTS["sim"][k]} (sim) / '
                               f'{MODE_DEFAULTS["real"][k]} (real)') for k in ARGS]
    return LaunchDescription(decl + [OpaqueFunction(function=_launch)])
