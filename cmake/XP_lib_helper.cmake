# ----------------------------------------------------------------------------
# This cmake helps to locate the pre-built XP libraries and set corresponding
# variables.
#
# [Output]
# This script will set the following variables:
#   XP_INCLUDE_DIR
#   XP_LIBRARY
#   TagDetector_INCLUDE_DIR
#   TagDetector_LIBRARY
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8.11)

# XP_LIBRARY is set here
find_library(XP_LIBRARY XP
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
set(XP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)

# TagDetector is set here
find_library(TagDetector_LIBRARY TagDetector
 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../lib_${CMAKE_SYSTEM_PROCESSOR}
)
set(TagDetector_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include/3rdparty/TagDetector/include)