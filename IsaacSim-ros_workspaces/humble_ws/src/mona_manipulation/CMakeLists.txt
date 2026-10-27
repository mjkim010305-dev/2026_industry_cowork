cmake_minimum_required(VERSION 3.8)
project(mona_manipulation)

find_package(ament_cmake REQUIRED)
find_package(mona_interfaces REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sweep_server.py
  scripts/sweep_only.py
  scripts/obstacle_clear_sequence.py
  scripts/moveit_to_isaac_bridge.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
