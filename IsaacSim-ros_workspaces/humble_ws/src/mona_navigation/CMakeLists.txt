cmake_minimum_required(VERSION 3.8)
project(mona_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mona_interfaces REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_library(mona_movable_obstacle_layer SHARED src/movable_obstacle_layer.cpp)
ament_target_dependencies(mona_movable_obstacle_layer
  nav2_costmap_2d pluginlib rclcpp std_msgs tf2)

pluginlib_export_plugin_description_file(nav2_costmap_2d movable_obstacle_layer_plugins.xml)

set(bt_plugin_libs)

add_library(mona_path_obstacle_detected_condition_bt_node SHARED
  src/path_obstacle_detected_condition.cpp)
ament_target_dependencies(mona_path_obstacle_detected_condition_bt_node
  rclcpp behaviortree_cpp_v3 nav2_behavior_tree mona_interfaces)
target_compile_definitions(mona_path_obstacle_detected_condition_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
list(APPEND bt_plugin_libs mona_path_obstacle_detected_condition_bt_node)

add_library(mona_judge_obstacle_action_bt_node SHARED
  src/judge_obstacle_action.cpp)
ament_target_dependencies(mona_judge_obstacle_action_bt_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_behavior_tree mona_interfaces)
target_compile_definitions(mona_judge_obstacle_action_bt_node
  PRIVATE BT_PLUGIN_EXPORT)
list(APPEND bt_plugin_libs mona_judge_obstacle_action_bt_node)

install(TARGETS mona_movable_obstacle_layer ${bt_plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params bt DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/path_obstacle_monitor.py DESTINATION lib/${PROJECT_NAME})

ament_package()
