# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_columbus_gz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED columbus_gz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(columbus_gz_FOUND FALSE)
  elseif(NOT columbus_gz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(columbus_gz_FOUND FALSE)
  endif()
  return()
endif()
set(_columbus_gz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT columbus_gz_FIND_QUIETLY)
  message(STATUS "Found columbus_gz: 0.0.1 (${columbus_gz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'columbus_gz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${columbus_gz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(columbus_gz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${columbus_gz_DIR}/${_extra}")
endforeach()
