# AI Festa Nav2 (ROS 2 Humble)

Uses the existing Isaac instance on domain 1. No custom BT, manipulation,
static TF, initial-pose publication, or goal publication is included.

## Build only this package

```bash
cd /home/students/workspace
colcon build --base-paths ai_festa_src --packages-select ai_festa_navigation --symlink-install
```

## Launch (do not launch a second copy if already running)

```bash
source /opt/ros/humble/setup.bash
source /home/students/workspace/install/local_setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ai_festa_navigation ai_festa_nav2.launch.py
```

Keep Isaac playing. The map is fixed to
`/home/students/workspace/src/ai-festa-map.yaml`.
All Nav2 nodes use simulation time. AMCL uses `base_footprint`; navigation
uses `base_link`; scan frame `base_scan` is resolved using Isaac's TF.
The local costmap uses a 2D obstacle layer instead of the original voxel layer.
Other baseline navigation tuning (including 0.22 m robot radius) is retained.

Velocity chain: controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel.
Standard recovery behaviors also publish /cmd_vel, as in Humble bringup.

## Initial pose is required to finish startup

AMCL deliberately has `set_initial_pose: false`. Before RViz 2D Pose Estimate,
AMCL cannot produce map -> odom. The standard Humble navigation lifecycle
manager waits while planner_server activates its global costmap; later nodes
may remain inactive until a valid initial pose and scan arrive. Do not add a
fake static map -> odom transform to hide this wait.

After setting a valid initial pose, check all lifecycle states are active
before sending the first goal. Goal execution is not tested by this package's
read-only diagnostic script:

```bash
ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 /home/students/workspace/ai_festa_diagnostics/verify_nav2.py
```

The verification result is saved to
`/home/students/workspace/ai_festa_diagnostics/latest_verification.json`.
