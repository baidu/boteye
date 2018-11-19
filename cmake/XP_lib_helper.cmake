# ----------------------------------------------------------------------------
# This cmake helps to locate the pre-built XP libraries and set corresponding
# variables.
#
# [Output]
# This script will set the following variables:
#   XP_INCLUDE_DIR
#   XP_LIBRARIES
#   XP_DRIVER_LIBRARY
#   TagDetector_INCLUDE_DIR
#   TagDetector_LIBRARY
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8.11)

# XP_LIBRARIES is set here
find_library(XP_LIBRARY XP
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
find_library(XP_PARAM_LIBRARY xpparam
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
find_library(XP_TIMER_LIBRARY xptimer
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
find_library(XP_DEPTH_LIBRARY xpdepth
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
set(XP_LIBRARIES ${XP_LIBRARY} ${XP_PARAM_LIBRARY} ${XP_TIMER_LIBRARY} ${XP_DEPTH_LIBRARY})
set(XP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# XP_DRIVER_LIBRARY is set here
find_library(XP_DRIVER_LIBRARY xpdriver
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)

# TagDetector is set here
find_library(TagDetector_LIBRARY TagDetector
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
set(TagDetector_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include/3rdparty/TagDetector/include)
