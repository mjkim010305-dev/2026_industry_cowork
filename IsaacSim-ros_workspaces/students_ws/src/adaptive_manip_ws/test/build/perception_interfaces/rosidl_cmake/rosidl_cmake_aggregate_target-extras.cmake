# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target perception_interfaces::perception_interfaces
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${perception_interfaces_TARGETS}.
if(perception_interfaces_TARGETS AND NOT TARGET perception_interfaces::perception_interfaces)
  add_library(perception_interfaces::perception_interfaces INTERFACE IMPORTED)
  set_target_properties(perception_interfaces::perception_interfaces PROPERTIES
    INTERFACE_LINK_LIBRARIES "${perception_interfaces_TARGETS}")
endif()
