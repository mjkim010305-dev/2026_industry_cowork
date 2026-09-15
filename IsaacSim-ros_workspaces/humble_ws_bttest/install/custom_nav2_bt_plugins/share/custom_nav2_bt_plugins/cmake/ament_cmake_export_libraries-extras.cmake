# generated from ament_cmake_export_libraries/cmake/template/ament_cmake_export_libraries.cmake.in

set(_exported_libraries "custom_is_movable_obstacle_detected_condition_bt_node;custom_wait_for_obstacle_clearance_action_bt_node;custom_capture_initial_path_action_bt_node;custom_approach_obstacle_normal_action_bt_node;custom_fixed_pick_object_action_bt_node;custom_is_obstacle_tracking_condition_bt_node;custom_lock_approach_pose_action_bt_node;custom_set_fixed_goal_action_bt_node;custom_place_object_action_bt_node;custom_topic_trigger_condition_bt_node;custom_push_obstacle_action_bt_node;custom_retract_arm_action_bt_node;custom_orient_path_to_yaw_action_bt_node;custom_open_loop_drive_action_bt_node;custom_record_odom_pose_action_bt_node;custom_record_map_pose_action_bt_node;custom_odom_limited_drive_action_bt_node;custom_drive_to_recorded_pose_action_bt_node;custom_compute_straight_path_to_pose_action_bt_node;custom_wiggle_in_place_action_bt_node;custom_movable_obstacle_localizers")
set(_exported_library_names "")

# populate custom_nav2_bt_plugins_LIBRARIES
if(NOT _exported_libraries STREQUAL "")
  # loop over libraries, either target names or absolute paths
  list(LENGTH _exported_libraries _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_libraries ${_i} _arg)

    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND custom_nav2_bt_plugins_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'custom_nav2_bt_plugins' passes the build configuration keyword '${_cfg}' as the last exported library")
      endif()
      list(GET _exported_libraries ${_i} _library)
    else()
      # the value is a library without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    if(NOT IS_ABSOLUTE "${_library}")
      # search for library target relative to this CMake file
      set(_lib "NOTFOUND")
      find_library(
        _lib NAMES "${_library}"
        PATHS "${custom_nav2_bt_plugins_DIR}/../../../lib"
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )

      if(NOT _lib)
        # warn about not existing library and ignore it
        message(FATAL_ERROR "Package 'custom_nav2_bt_plugins' exports the library '${_library}' which couldn't be found")
      elseif(NOT IS_ABSOLUTE "${_lib}")
        # the found library must be an absolute path
        message(FATAL_ERROR "Package 'custom_nav2_bt_plugins' found the library '${_library}' at '${_lib}' which is not an absolute path")
      elseif(NOT EXISTS "${_lib}")
        # the found library must exist
        message(FATAL_ERROR "Package 'custom_nav2_bt_plugins' found the library '${_lib}' which doesn't exist")
      else()
        list(APPEND custom_nav2_bt_plugins_LIBRARIES ${_cfg} "${_lib}")
      endif()

    else()
      if(NOT EXISTS "${_library}")
        # the found library must exist
        message(WARNING "Package 'custom_nav2_bt_plugins' exports the library '${_library}' which doesn't exist")
      else()
        list(APPEND custom_nav2_bt_plugins_LIBRARIES ${_cfg} "${_library}")
      endif()
    endif()
  endwhile()
endif()

# find_library() library names with optional LIBRARY_DIRS
# and add the libraries to custom_nav2_bt_plugins_LIBRARIES
if(NOT _exported_library_names STREQUAL "")
  # loop over library names
  # but remember related build configuration keyword if available
  list(LENGTH _exported_library_names _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_library_names ${_i} _arg)
    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND custom_nav2_bt_plugins_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library name
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'custom_nav2_bt_plugins' passes the build configuration keyword '${_cfg}' as the last exported target")
      endif()
      list(GET _exported_library_names ${_i} _library)
    else()
      # the value is a library target without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    # extract optional LIBRARY_DIRS from library name
    string(REPLACE ":" ";" _library_dirs "${_library}")
    list(GET _library_dirs 0 _library_name)
    list(REMOVE_AT _library_dirs 0)

    set(_lib "NOTFOUND")
    if(NOT _library_dirs)
      # search for library in the common locations
      find_library(
        _lib
        NAMES "${_library_name}"
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING "Package 'custom_nav2_bt_plugins' exports library '${_library_name}' which couldn't be found")
      endif()
    else()
      # search for library in the specified directories
      find_library(
        _lib
        NAMES "${_library_name}"
        PATHS ${_library_dirs}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING
          "Package 'custom_nav2_bt_plugins' exports library '${_library_name}' with LIBRARY_DIRS '${_library_dirs}' which couldn't be found")
      endif()
    endif()
    if(_lib)
      list(APPEND custom_nav2_bt_plugins_LIBRARIES ${_cfg} "${_lib}")
    endif()
  endwhile()
endif()

# TODO(dirk-thomas) deduplicate custom_nav2_bt_plugins_LIBRARIES
# while maintaining library order
# as well as build configuration keywords
# as well as linker flags
