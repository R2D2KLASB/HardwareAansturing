# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sound_player_node_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sound_player_node_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sound_player_node_FOUND FALSE)
  elseif(NOT sound_player_node_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sound_player_node_FOUND FALSE)
  endif()
  return()
endif()
set(_sound_player_node_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sound_player_node_FIND_QUIETLY)
  message(STATUS "Found sound_player_node: 0.0.0 (${sound_player_node_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sound_player_node' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sound_player_node_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sound_player_node_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sound_player_node_DIR}/${_extra}")
endforeach()
