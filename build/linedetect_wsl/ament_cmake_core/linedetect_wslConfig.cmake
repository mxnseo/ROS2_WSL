# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_linedetect_wsl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED linedetect_wsl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(linedetect_wsl_FOUND FALSE)
  elseif(NOT linedetect_wsl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(linedetect_wsl_FOUND FALSE)
  endif()
  return()
endif()
set(_linedetect_wsl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT linedetect_wsl_FIND_QUIETLY)
  message(STATUS "Found linedetect_wsl: 0.0.0 (${linedetect_wsl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'linedetect_wsl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${linedetect_wsl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(linedetect_wsl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${linedetect_wsl_DIR}/${_extra}")
endforeach()
