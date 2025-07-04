# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_shortest_path_viz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED shortest_path_viz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(shortest_path_viz_FOUND FALSE)
  elseif(NOT shortest_path_viz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(shortest_path_viz_FOUND FALSE)
  endif()
  return()
endif()
set(_shortest_path_viz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT shortest_path_viz_FIND_QUIETLY)
  message(STATUS "Found shortest_path_viz: 0.0.0 (${shortest_path_viz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'shortest_path_viz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${shortest_path_viz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(shortest_path_viz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${shortest_path_viz_DIR}/${_extra}")
endforeach()
