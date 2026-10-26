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

include_directories(include)

add_library(mona_movable_obstacle_layer SHARED src/movable_obstacle_layer.cpp)
ament_target_dependencies(mona_movable_obstacle_layer
  nav2_costmap_2d pluginlib rclcpp std_msgs tf2)

pluginlib_export_plugin_description_file(nav2_costmap_2d movable_obstacle_layer_plugins.xml)

install(TARGETS mona_movable_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_package()
