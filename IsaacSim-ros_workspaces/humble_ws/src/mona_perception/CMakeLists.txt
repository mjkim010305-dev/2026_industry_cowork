cmake_minimum_required(VERSION 3.8)
project(mona_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/vlm_movability_node.py
  scripts/movable_obstacle_map_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
