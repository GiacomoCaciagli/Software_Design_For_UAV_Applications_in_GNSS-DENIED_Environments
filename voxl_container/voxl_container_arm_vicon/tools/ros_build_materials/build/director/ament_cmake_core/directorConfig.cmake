# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_director_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED director_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(director_FOUND FALSE)
  elseif(NOT director_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(director_FOUND FALSE)
  endif()
  return()
endif()
set(_director_CONFIG_INCLUDED TRUE)

# output package information
if(NOT director_FIND_QUIETLY)
  message(STATUS "Found director: 0.0.0 (${director_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'director' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${director_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(director_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${director_DIR}/${_extra}")
endforeach()
