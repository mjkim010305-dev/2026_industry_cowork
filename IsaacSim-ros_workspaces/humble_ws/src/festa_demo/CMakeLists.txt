cmake_minimum_required(VERSION 3.8)
project(festa_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

# ---------------------------------------------------------------------------
# Sweep.action - self-contained copy of
# src/festa_manipulation/turtlebot3_msgs/action/Sweep.action
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Sweep.action"
)

# ---------------------------------------------------------------------------
# BT plugin: IsGreenBoxDetected (condition) - fresh detection gate for the
# AI Festa green-box approach-and-stop demo
# (copied from custom_nav2_bt_plugins/is_green_box_detected_condition)
# ---------------------------------------------------------------------------
add_library(festa_demo_is_green_box_detected_bt_node SHARED
  src/is_green_box_detected_condition.cpp)
ament_target_dependencies(festa_demo_is_green_box_detected_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree std_msgs geometry_msgs)
target_compile_definitions(festa_demo_is_green_box_detected_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: ComputeGreenBoxApproachGoal (sync action) - box pose -> standoff
# goal in front of the box, for ComputePathToPose/FollowPath
# (copied from custom_nav2_bt_plugins/compute_green_box_approach_goal_action)
# ---------------------------------------------------------------------------
add_library(festa_demo_compute_green_box_approach_goal_bt_node SHARED
  src/compute_green_box_approach_goal_action.cpp)
ament_target_dependencies(festa_demo_compute_green_box_approach_goal_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree nav2_util geometry_msgs tf2 tf2_ros)
target_compile_definitions(festa_demo_compute_green_box_approach_goal_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: FinalApproachStop (stateful action) - open-loop /cmd_vel creep,
# stops on min /scan range in the forward cone (bypasses costmap, see header)
# (copied from custom_nav2_bt_plugins/final_approach_stop_action)
# ---------------------------------------------------------------------------
add_library(festa_demo_final_approach_stop_bt_node SHARED
  src/final_approach_stop_action.cpp)
ament_target_dependencies(festa_demo_final_approach_stop_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree sensor_msgs geometry_msgs
  nav2_util tf2 tf2_ros)
target_compile_definitions(festa_demo_final_approach_stop_bt_node
  PRIVATE BT_PLUGIN_EXPORT)

# ---------------------------------------------------------------------------
# BT plugin: SweepObstacle (stateful action) - /sweep action client
# (festa_demo/action/Sweep, this package's own interface)
# (copied from custom_nav2_bt_plugins/sweep_obstacle_action)
# ---------------------------------------------------------------------------
add_library(festa_demo_sweep_obstacle_bt_node SHARED
  src/sweep_obstacle_action.cpp)
ament_target_dependencies(festa_demo_sweep_obstacle_bt_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_behavior_tree)
rosidl_get_typesupport_target(festa_demo_sweep_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(festa_demo_sweep_obstacle_bt_node "${festa_demo_sweep_typesupport}")
target_compile_definitions(festa_demo_sweep_obstacle_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
add_dependencies(festa_demo_sweep_obstacle_bt_node ${PROJECT_NAME})

set(bt_plugin_libs
  festa_demo_is_green_box_detected_bt_node
  festa_demo_compute_green_box_approach_goal_bt_node
  festa_demo_final_approach_stop_bt_node
  festa_demo_sweep_obstacle_bt_node
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS ${bt_plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/detector_node.py
  scripts/geometry.py
  scripts/obstacle_clear_sequence.py
  scripts/sweep_only.py
  scripts/sweep_action_server.py
  scripts/send_goal.py
  scripts/moveit_to_isaac_bridge.py
  scripts/visual_approach.py
  scripts/box_on_path.py
  scripts/rear_pick.py
  scripts/push_through.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY bt
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${bt_plugin_libs})
ament_export_dependencies(
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_behavior_tree nav2_util
  std_msgs sensor_msgs geometry_msgs tf2 tf2_ros rosidl_default_runtime)

ament_package()
