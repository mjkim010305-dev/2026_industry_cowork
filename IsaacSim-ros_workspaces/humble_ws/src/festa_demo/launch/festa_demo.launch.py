"""AI Festa L-course scenario, self-contained (festa_demo).

    ros2 launch festa_demo festa_demo.launch.py mode:=sim    # Isaac already playing
    ros2 launch festa_demo festa_demo.launch.py mode:=real   # robot bringup already up

Derived from festa_bringup/launch/festa_scenario.launch.py's pick:=false path:
no `pick` arg (this package never carries a part), and every script that
festa_scenario.launch.py found under `ws_src` (bridge, sweep server, sweep
sequence, detector, send_goal) is installed into THIS package instead, so
there is no `ws_src` arg either.

Starts, in order:
  1. (sim only) moveit_to_isaac_bridge.py - /arm_controller and /gripper_controller
     action servers on top of Isaac's /joint_commands (on the real robot the
     ros2_control controllers from hardware.launch.py provide them).
  2. (sim only, publish_camera_tf) base_link -> camera_optical static TF.
  3. Nav2 bringup (map + params, BT = bt/festa_demo.xml).
  4. detector_node.py (HSV + lidar) -> green_box/pose, green_box/detected.
  5. sweep_action_server.py -> /sweep (safety cutoff on for real, off for sim:
     Isaac has no joint current). Runs scripts/sweep_only.py: no part, sweep
     from P_HOME.
  6. After goal_delay s: send_goal - initial pose, the goal, then (return_to_start)
     back to the initial pose.

The robot bringup itself (Isaac, or hardware.launch.py + lidar + camera on the
robot) is NOT part of this launch.
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

PKG = 'festa_demo'

# Per-mode defaults; any of them can be overridden on the command line.
MODE_DEFAULTS = {
    'sim': {
        'map': 'l_course_sim.yaml',
        'image_topic': '/front_camera',
        'camera_info_topic': '/front_camera_info',
        'camera_frame': 'camera_optical',
        'publish_camera_tf': 'true',
        # base_link -> front_camera as Isaac itself publishes it (/tf: base_link -> rsd455
        # (0.073, -0.0007, 0.0947), rsd455 -> front_camera (0, -0.0115, 0), level), measured
        # 2026-10-01. The old festa_scenario value (0.017, 0, 0.461, 8.6 deg down) was wrong.
        'camera_tf': '0.073 -0.0122 0.0947 -0.5 0.5 -0.5 0.5',
        'safety_monitor': 'false',
        'isaac_bridge': 'true',
        'stamp_on_receive': 'false',
        'range_from_image': 'false',
        # Arrive facing the direction of travel (no big turn at the goal); the
        # 180 deg turn for the way back is done in place by the rotation shim.
        'goal_x': '1.75', 'goal_y': '-1.75', 'goal_yaw': '-1.5708',
    },
    'real': {
        'map': 'l_course_real.yaml',
        'image_topic': '/camera/camera/color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        # festa_demo (2026-10-01): the RealSense node hangs its colour frame off the URDF's
        # TB3 R200 camera_link, i.e. (0.073, -0.070, 0.084) - not where the D555 is. Use our
        # own optical frame at the D555 colour sensor instead: the team URDF's d555_link
        # (0.1205, 0.0475, 0.0624; 0.0475 = D4xx mount centre -> camera_link) plus the
        # camera's own camera_link -> colour offset (0, -0.059, 0) = 1.2 cm right of the
        # centre line (user measured ~19 cm from the body centre, ~5 cm above the floor).
        'camera_frame': 'festa_color_optical',
        'publish_camera_tf': 'true',
        'camera_tf': '0.1205 -0.0115 0.0624 -0.5 0.5 -0.5 0.5',
        'safety_monitor': 'true',
        'isaac_bridge': 'false',
        # The robot's RealSense image stamps drift away from the system clock.
        'stamp_on_receive': 'true',
        # Box position from its image width only (lidar for walls/obstacles).
        'range_from_image': 'true',
        # Robot's own real.yaml (2026-09-30 mapping): (0,0) is the start at the open
        # end of the horizontal leg facing +x; the goal is the bottom of the
        # vertical leg (x 1.15..2.65, floor y -2.05), facing down the leg.
        'goal_x': '1.9', 'goal_y': '-1.7', 'goal_yaw': '-1.5708',
    },
}
ARGS = ['map', 'image_topic', 'camera_info_topic', 'camera_frame', 'publish_camera_tf', 'camera_tf',
        'safety_monitor', 'isaac_bridge', 'goal_x', 'goal_y', 'goal_yaw', 'stamp_on_receive',
        'range_from_image']


def _launch(context):
    share = get_package_share_directory(PKG)
    pick = LaunchConfiguration('pick').perform(context) == 'true'
    # pick_first:=false - the part is already on the robot's back (P_REAR_CARRY).
    pick_first = pick and LaunchConfiguration('pick_first').perform(context) != 'false'
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODE_DEFAULTS:
        raise RuntimeError(f"mode must be 'sim' or 'real', got '{mode}'")
    cfg = {k: LaunchConfiguration(k).perform(context) or MODE_DEFAULTS[mode][k] for k in ARGS}
    sim = mode == 'sim'
    use_sim_time = 'true' if sim else 'false'
    map_yaml = cfg['map'] if os.path.isabs(cfg['map']) else os.path.join(share, 'maps', cfg['map'])
    bt_xml = LaunchConfiguration('bt_xml').perform(context) or os.path.join(share, 'bt', 'festa_demo.xml')
    params = LaunchConfiguration('params_file').perform(context) or os.path.join(
        share, 'params', 'festa_demo_nav2.yaml')
    lib = os.path.join(get_package_prefix(PKG), 'lib', PKG)
    detector = os.path.join(lib, 'detector_node.py')
    sweep_server = os.path.join(lib, 'sweep_action_server.py')
    send_goal = os.path.join(lib, 'send_goal.py')
    bridge = os.path.join(lib, 'moveit_to_isaac_bridge.py')
    for path in [map_yaml, bt_xml, params, detector, sweep_server, send_goal] + (
            [bridge] if cfg['isaac_bridge'] == 'true' else []):
        if not os.path.isfile(path):
            raise RuntimeError(f'festa_demo: missing {path}')

    actions = []
    if cfg['isaac_bridge'] == 'true':
        actions.append(ExecuteProcess(
            name='isaac_arm_bridge', output='screen',
            cmd=['python3', bridge, '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', 'relay_joint_states:=false', '-p', 'isaac_state_topic:=/joint_states',
                 '-p', 'isaac_command_topic:=/joint_commands']))
    if cfg['publish_camera_tf'] == 'true':
        x, y, z, qx, qy, qz, qw = cfg['camera_tf'].split()
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf',
            arguments=['--x', x, '--y', y, '--z', z, '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
                       '--frame-id', 'base_link', '--child-frame-id', cfg['camera_frame']],
            parameters=[{'use_sim_time': sim}]))

    nav2_params = RewrittenYaml(source_file=params, param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
                                convert_types=True)
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml, 'params_file': nav2_params, 'use_sim_time': use_sim_time,
                          'autostart': 'true'}.items()))

    actions.append(ExecuteProcess(
        name='green_box_detector', output='screen',
        cmd=['python3', detector, '--ros-args',
             '-p', f'use_sim_time:={str(sim).lower()}',
             '-p', f"image_topic:={cfg['image_topic']}",
             '-p', f"camera_info_topic:={cfg['camera_info_topic']}",
             '-p', 'scan_topic:=/scan',
             '-p', f"camera_frame:={cfg['camera_frame']}",
             '-p', 'output_frame:=map',
             '-p', f"box_depth_m:={LaunchConfiguration('box_depth').perform(context)}",
             '-p', f"box_height_m:={LaunchConfiguration('box_height').perform(context)}",
             '-p', f"stamp_on_receive:={cfg['stamp_on_receive']}",
             '-p', f"range_from_image:={cfg['range_from_image']}",
             '-p', f"box_face_width_m:={LaunchConfiguration('box_face_width').perform(context)}",
             # One frame is enough (user, 2026-10-01: detection is accurate; on the
             # loaded Pi three agreeing frames took long and image ranging while
             # turning kept resetting the window as "unstable").
             '-p', 'confirm_frames:=1',
             '-p', 'max_rate_hz:=5.0']))

    actions.append(ExecuteProcess(
        name='push_through', output='screen',
        cmd=['python3', os.path.join(lib, 'push_through.py'), '--ros-args',
             '-p', f'use_sim_time:={str(sim).lower()}'] +
        # festa_demo (2026-10-01, sim S1e): the time allowances are wall-clock and Isaac
        # runs at ~1/3 real time, so a 15 s push got ~5 sim-seconds.
        (['-p', 'time_allowance:=60.0'] if sim else [])))

    actions.append(ExecuteProcess(
        name='box_on_path', output='screen',
        cmd=['python3', os.path.join(lib, 'box_on_path.py'), '--ros-args',
             '-p', f'use_sim_time:={str(sim).lower()}',
             '-p', f"box_face_width_m:={LaunchConfiguration('box_face_width').perform(context)}",
             '-p', f"box_height_m:={LaunchConfiguration('box_height').perform(context)}",
             '-p', f"camera_info_topic:={cfg['camera_info_topic']}",
             # festa_demo (2026-10-02): camera position in base_link for the box position
             '-p', f"camera_x:={cfg['camera_tf'].split()[0]}",
             '-p', f"camera_y:={cfg['camera_tf'].split()[1]}"]))

    actions.append(ExecuteProcess(
        name='visual_approach', output='screen',
        cmd=['python3', os.path.join(lib, 'visual_approach.py'), '--ros-args',
             '-p', f'use_sim_time:={str(sim).lower()}'] +
        (['-p', 'time_allowance:=240.0'] if sim else [])))

    actions.append(ExecuteProcess(
        name='sweep_action_server', output='screen',
        cmd=['python3', sweep_server, '--ros-args', '-p', f"safety_monitor:={cfg['safety_monitor']}",
             '-p', 'sequence:=' + ('obstacle_clear_sequence' if pick else 'sweep_only')]))

    actions.append(TimerAction(
        period=float(LaunchConfiguration('goal_delay').perform(context)),
        actions=[ExecuteProcess(
            name='festa_send_goal', output='screen',
            cmd=['python3', send_goal, '--ros-args',
                 '-p', f'use_sim_time:={str(sim).lower()}',
                 '-p', f"goal_x:={cfg['goal_x']}", '-p', f"goal_y:={cfg['goal_y']}",
                 '-p', f"goal_yaw:={cfg['goal_yaw']}",
                 '-p', f"set_initial_pose:={LaunchConfiguration('set_initial_pose').perform(context)}",
                 '-p', f"return_to_start:={LaunchConfiguration('return_to_start').perform(context)}",
                 '-p', f"round_trips:={LaunchConfiguration('round_trips').perform(context)}",
                 '-p', f"pick_first:={str(pick_first).lower()}",
                 '-p', f"send_goal:={LaunchConfiguration('send_goal').perform(context)}",
                 '-p', f"initial_x:={LaunchConfiguration('initial_x').perform(context)}",
                 '-p', f"initial_y:={LaunchConfiguration('initial_y').perform(context)}",
                 '-p', f"initial_yaw:={LaunchConfiguration('initial_yaw').perform(context)}"])]))
    return actions


def generate_launch_description():
    decl = [
        DeclareLaunchArgument('mode', default_value='sim', description="'sim' (Isaac) or 'real' (robot)"),
        DeclareLaunchArgument('bt_xml', default_value='',
                              description='BT xml (default: bt/festa_demo.xml)'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 params (default: params/festa_demo_nav2.yaml)'),
        DeclareLaunchArgument('box_depth', default_value='0.185', description='green box depth [m] (detector)'),
        DeclareLaunchArgument('box_height', default_value='0.12', description='green box height [m] (detector)'),
        DeclareLaunchArgument('pick', default_value='true',
                              description="true: load the part first (robot's rear_pick.py) and run the "
                                          "manipulation team's obstacle_clear_sequence.py at the box "
                                          "(put down behind -> sweep -> pick up again); false: sweep only"),
        DeclareLaunchArgument('send_goal', default_value='true',
                              description='false: start Nav2 and set the initial pose only (no pick, no goal)'),
        DeclareLaunchArgument('pick_first', default_value='true',
                              description='with pick:=true, false skips the initial rear_pick.py '
                                          '(part already at P_REAR_CARRY)'),
        DeclareLaunchArgument('box_face_width', default_value='0.24',
                              description='apparent box width in the image [m] for image ranging '
                                          '(real upright box 2026-10-01: 0.185 m face looked ~0.24 m)'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('return_to_start', default_value='true',
                              description='after reaching the goal, drive back to initial_x/y/yaw'),
        DeclareLaunchArgument('round_trips', default_value='1',
                              description='goal -> start round trips (0 = forever, AI Festa shuttle)'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_delay', default_value='20.0',
                              description='seconds before the initial pose / goal are sent'),
    ] + [DeclareLaunchArgument(k, default_value='', description=f'per-mode default: {MODE_DEFAULTS["sim"][k]} (sim) / '
                               f'{MODE_DEFAULTS["real"][k]} (real)') for k in ARGS]
    return LaunchDescription(decl + [OpaqueFunction(function=_launch)])
