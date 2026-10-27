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

"""VLM movability perception: vlm_movability_node + movable_obstacle_map_node
+ the camera optical static TF.

Merged from the sim launch files vlm_movability.launch.py and
vlm_perception.launch.py. ``mode:=sim`` reproduces them (camera TF since
corrected, see MODE_DEFAULTS); ``mode:=real``
swaps in the festa_demo real-robot camera frame/TF and PLACEHOLDER topics that
must be checked against the real camera driver before use.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Per-mode defaults. Real values marked PLACEHOLDER are NOT verified.
MODE_DEFAULTS = {
    "sim": {
        "rgb_topic": "/front_camera",
        "depth_topic": "/depth_camera",
        "camera_info_topic": "/depth_camera_info",
        "use_sim_time": "true",
        # The scene publishes base_link -> sim_camera as the identity, which
        # discards the camera mounting AND treats optical-convention depth
        # points (x right, y down, z forward) as if they were body-frame. That
        # writes the forward distance into the map Z axis: an obstacle 1.4 m
        # ahead came out at the robot's own feet, and the robot drove at itself.
        #
        # The camera Isaac renders from is the RSD455 colour sensor
        # (.../rsd455/RSD455/Camera_OmniVision_OV9782_Color, fx 634.09 =
        # camera_info), not /base_link/Camera (0.46 m up, 8.6 deg down) whose
        # pose this TF used to copy. Everything below is MEASURED on the live
        # frames, not read from the USD.
        #
        # x: with the robot parked at x=28.70 facing a box whose front face is
        # at x=30.12, the centre depth pixel reads 1.403 m, so the camera sits
        # +0.017 m from base_link.
        # z and rotation: a plane fit to the floor in the live depth
        # (perc_j1 round 1) puts the camera 0.106 m above the floor and level
        # (pitch -0.2 deg); base_link spawns 0.002 m above the floor. Level
        # optical axes = the standard base -> optical rotation. The old 0.46 m
        # / 8.6 deg pose moved the on-demand SAM prompt points 129 px down at
        # 1 m (onto the floor under a 0.5 m box).
        "optical_frame": "camera_optical",
        "tf": ["0.017", "0.0", "0.104",
               "-0.5", "0.5", "-0.5", "0.5"],
    },
    "real": {
        # PLACEHOLDER: replace with the real camera driver's topics.
        "rgb_topic": "/camera/color/image_raw",
        "depth_topic": "/PLACEHOLDER/depth/image_rect_raw",
        "camera_info_topic": "/camera/depth/camera_info",
        "use_sim_time": "false",
        # Same frame/TF festa_demo publishes for the D555 colour sensor.
        "optical_frame": "festa_color_optical",
        "tf": ["0.1205", "-0.0115", "0.0624",
               "-0.5", "0.5", "-0.5", "0.5"],
    },
}


def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in MODE_DEFAULTS:
        raise RuntimeError("mode must be 'sim' or 'real', got %r" % mode)
    d = MODE_DEFAULTS[mode]

    def arg(name):
        value = LaunchConfiguration(name).perform(context)
        return value if value != "" else d[name]

    # Already a plain string here, and ParameterValue type-checks plain
    # values instead of converting them, so turn it into a bool directly.
    use_sim_time = arg("use_sim_time").lower() == "true"
    trigger = LaunchConfiguration("trigger").perform(context)
    if trigger not in ("periodic", "on_demand"):
        raise RuntimeError("trigger must be 'periodic' or 'on_demand', got %r" % trigger)
    judge_backend = LaunchConfiguration("judge_backend").perform(context)
    if judge_backend not in ("ollama", "logit"):
        raise RuntimeError(
            "judge_backend must be 'ollama' or 'logit', got %r" % judge_backend)
    x, y, z, qx, qy, qz, qw = d["tf"]

    vlm_params = {
        "use_sim_time": use_sim_time,
        "rgb_topic": arg("rgb_topic"),
        "depth_topic": arg("depth_topic"),
        "camera_info_topic": arg("camera_info_topic"),
        "movable_topic": "/obstacle/is_movable",
        "roi_topic": "/vlm/roi",
        "annotated_topic": "/vlm/annotated",
        "judgments_topic": "/vlm/judgments",
        "reset_topic": "/vlm/reset",
        "ollama_url": LaunchConfiguration("ollama_url"),
        "model": LaunchConfiguration("model"),
        # A thinking model needs room for the reasoning AND the JSON;
        # too small a budget returns an empty string with no error.
        "num_predict": 2000,
        "request_timeout": 60.0,
        "max_retries": 2,
        "query_period": ParameterValue(
            LaunchConfiguration("query_period"), value_type=float),
        "publish_rate": 5.0,
        "latch": ParameterValue(LaunchConfiguration("latch"), value_type=bool),
        "detector_url": LaunchConfiguration("detector_url"),
        "trigger": trigger,
        "sam_url": LaunchConfiguration("sam_url"),
        "judge_backend": judge_backend,
        "judge_url": LaunchConfiguration("judge_url"),
        # Segmentation and push-size thresholds are deliberately NOT repeated
        # here: tuned values live in the node next to the reasoning for them;
        # override from the command line when experimenting.
    }
    if mode == "real":
        # sim relies on the node default (camera_optical).
        vlm_params["optical_frame"] = d["optical_frame"]

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_optical_tf",
            output="screen",
            arguments=[
                "--x", x, "--y", y, "--z", z,
                "--qx", qx, "--qy", qy, "--qz", qz, "--qw", qw,
                "--frame-id", "base_link",
                "--child-frame-id", d["optical_frame"],
            ],
        ),
        Node(
            package="mona_perception",
            executable="vlm_movability_node.py",
            name="vlm_movability",
            output="screen",
            parameters=[vlm_params],
        ),
    ]
    # on_demand judges one obstacle per request; nothing feeds the semantic map.
    if trigger == "periodic":
        nodes.append(
            Node(
                package="mona_perception",
                executable="movable_obstacle_map_node.py",
                name="movable_obstacle_map",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "semantic_map_path": LaunchConfiguration("semantic_map_path"),
                }],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="sim",
            description="sim | real. Selects topic/frame/TF/use_sim_time defaults."),
        # Empty = take the value from the selected mode's defaults.
        DeclareLaunchArgument("use_sim_time", default_value="",
                              description="empty: true for sim, false for real"),
        DeclareLaunchArgument("rgb_topic", default_value="",
                              description="empty: mode default"),
        DeclareLaunchArgument("depth_topic", default_value="",
                              description="empty: mode default (real is a PLACEHOLDER)"),
        DeclareLaunchArgument("camera_info_topic", default_value="",
                              description="empty: mode default"),
        DeclareLaunchArgument("semantic_map_path",
                              default_value=os.path.expanduser("~/mona_semantic_map.yaml")),
        DeclareLaunchArgument("model", default_value="gemma4:e4b"),
        DeclareLaunchArgument("ollama_url",
                              default_value="http://127.0.0.1:11434/api/generate",
                              description="Ollama endpoint, e.g. the lab server"),
        DeclareLaunchArgument("query_period", default_value="2.0"),
        # Latching holds the verdict once a movable target is picked, so a push
        # already under way is never contradicted by a later frame.
        DeclareLaunchArgument("latch", default_value="true"),
        # Empty = depth-only candidates. See the node docstring: the detector
        # measured on these scenes classified worse than the language model.
        DeclareLaunchArgument("detector_url", default_value=""),
        # periodic: 2 s VLM loop + semantic map (sim/harness default).
        # on_demand: JudgeObstacle action server (/judge_obstacle) only, no map node.
        DeclareLaunchArgument("trigger", default_value="periodic",
                              description="periodic | on_demand"),
        DeclareLaunchArgument("sam_url", default_value="http://127.0.0.1:8091",
                              description="SAM segmentation server (on_demand)"),
        DeclareLaunchArgument("judge_backend", default_value="ollama",
                              description="ollama | logit (on_demand judge)"),
        DeclareLaunchArgument("judge_url", default_value="http://127.0.0.1:8092",
                              description="logit judge server (judge_backend:=logit)"),
        OpaqueFunction(function=_launch_setup),
    ])
