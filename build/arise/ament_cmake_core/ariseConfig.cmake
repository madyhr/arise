# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arise_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arise_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arise_FOUND FALSE)
  elseif(NOT arise_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arise_FOUND FALSE)
  endif()
  return()
endif()
set(_arise_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arise_FIND_QUIETLY)
  message(STATUS "Found arise: 0.0.0 (${arise_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arise' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT arise_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arise_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${arise_DIR}/${_extra}")
endforeach()
