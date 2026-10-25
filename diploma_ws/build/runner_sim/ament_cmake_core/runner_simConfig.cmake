# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_runner_sim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED runner_sim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(runner_sim_FOUND FALSE)
  elseif(NOT runner_sim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(runner_sim_FOUND FALSE)
  endif()
  return()
endif()
set(_runner_sim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT runner_sim_FIND_QUIETLY)
  message(STATUS "Found runner_sim: 0.1.0 (${runner_sim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'runner_sim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT runner_sim_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(runner_sim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "rosidl_cmake-extras.cmake;ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake;rosidl_cmake_export_typesupport_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake;rosidl_cmake_export_typesupport_libraries-extras.cmake;rosidl_cmake_aggregate_target-extras.cmake")
foreach(_extra ${_extras})
  include("${runner_sim_DIR}/${_extra}")
endforeach()
