cmake_minimum_required(VERSION 3.8)
project(custom_nav2_bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# BT plugin: IsMovableObstacleDetected (condition) - VLM std_msgs/Bool
# ---------------------------------------------------------------------------
add_library(custom_is_movable_obstacle_detected_condition_bt_node SHARED
  src/is_movable_obstacle_detected_condition.cpp)
ament_target_dependencies(custom_is_movable_obstacle_detected_condition_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree std_msgs)
target_compile_definitions(custom_is_movable_obstacle_detected_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: WaitForObstacleClearance (stateful action) - VLM std_msgs/Bool
# ---------------------------------------------------------------------------
add_library(custom_wait_for_obstacle_clearance_action_bt_node SHARED
  src/wait_for_obstacle_clearance_action.cpp)
ament_target_dependencies(custom_wait_for_obstacle_clearance_action_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree std_msgs)
target_compile_definitions(custom_wait_for_obstacle_clearance_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: CaptureInitialPath (sync action) - latch the pre-detour path
# ---------------------------------------------------------------------------
add_library(custom_capture_initial_path_action_bt_node SHARED
  src/capture_initial_path_action.cpp)
ament_target_dependencies(custom_capture_initial_path_action_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree nav_msgs geometry_msgs)
target_compile_definitions(custom_capture_initial_path_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: ApproachObstacleNormal (sync action) - straight-in approach path
# ---------------------------------------------------------------------------
add_library(custom_approach_obstacle_normal_action_bt_node SHARED
  src/approach_obstacle_normal_action.cpp)
ament_target_dependencies(custom_approach_obstacle_normal_action_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree nav2_util nav_msgs geometry_msgs tf2 tf2_ros)
target_compile_definitions(custom_approach_obstacle_normal_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: FixedPickObject (stateful action) - one-shot fixed pick sequence
# ---------------------------------------------------------------------------
add_library(custom_fixed_pick_object_action_bt_node SHARED
  src/fixed_pick_object_action.cpp)
ament_target_dependencies(custom_fixed_pick_object_action_bt_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_behavior_tree
  control_msgs trajectory_msgs)
target_compile_definitions(custom_fixed_pick_object_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: IsObstacleTracking (condition) - stabilised /perception_state gate
# ---------------------------------------------------------------------------
add_library(custom_is_obstacle_tracking_condition_bt_node SHARED
  src/is_obstacle_tracking_condition.cpp)
ament_target_dependencies(custom_is_obstacle_tracking_condition_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree std_msgs)
target_compile_definitions(custom_is_obstacle_tracking_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: LockApproachPose (stateful action) - freeze the obstacle pose once
# ---------------------------------------------------------------------------
add_library(custom_lock_approach_pose_action_bt_node SHARED
  src/lock_approach_pose_action.cpp)
ament_target_dependencies(custom_lock_approach_pose_action_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree geometry_msgs)
target_compile_definitions(custom_lock_approach_pose_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

set(bt_plugin_libs
  custom_is_movable_obstacle_detected_condition_bt_node
  custom_wait_for_obstacle_clearance_action_bt_node
  custom_capture_initial_path_action_bt_node
  custom_approach_obstacle_normal_action_bt_node
  custom_fixed_pick_object_action_bt_node
  custom_is_obstacle_tracking_condition_bt_node
  custom_lock_approach_pose_action_bt_node
)

# ---------------------------------------------------------------------------
# pluginlib: MovableObstacleLocalizer implementations (RGB-D -> obstacle point)
# ---------------------------------------------------------------------------
add_library(custom_movable_obstacle_localizers SHARED
  src/localizers/depth_centroid_localizer.cpp)
ament_target_dependencies(custom_movable_obstacle_localizers
  rclcpp pluginlib sensor_msgs)
pluginlib_export_plugin_description_file(custom_nav2_bt_plugins
  movable_obstacle_localizer_plugins.xml)

# ---------------------------------------------------------------------------
# Host node: runs a MovableObstacleLocalizer plugin, TF -> map, publishes pose
# ---------------------------------------------------------------------------
add_executable(rgbd_obstacle_localizer
  src/rgbd_obstacle_localizer_node.cpp)
ament_target_dependencies(rgbd_obstacle_localizer
  rclcpp pluginlib std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS ${bt_plugin_libs} custom_movable_obstacle_localizers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS rgbd_obstacle_localizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Python node: Perception Stabilizer (VLM detect + depth -> stabilised estimate)
install(PROGRAMS
  scripts/perception_stabilizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES movable_obstacle_localizer_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${bt_plugin_libs} custom_movable_obstacle_localizers)
ament_export_dependencies(
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_behavior_tree nav2_util pluginlib
  std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
  control_msgs trajectory_msgs)

ament_package()
