# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target custom_message::custom_message
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${custom_message_TARGETS}.
if(custom_message_TARGETS AND NOT TARGET custom_message::custom_message)
  add_library(custom_message::custom_message INTERFACE IMPORTED)
  set_target_properties(custom_message::custom_message PROPERTIES
    INTERFACE_LINK_LIBRARIES "${custom_message_TARGETS}")
endif()
