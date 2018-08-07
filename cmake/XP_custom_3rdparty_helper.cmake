# ----------------------------------------------------------------------------
# This cmake helps to locate the pre-built XP custom 3rdparty dependencies
# and set corresponding variables, including:
# OpenCV, (Optional: Json)
# ----------------------------------------------------------------------------
# [IMPORTANT] The provided 3rdparty_lib_lean has to be extracted exactly to
# $ENV{HOME}/XP_release/3rdparty_lib_lean
cmake_minimum_required(VERSION 2.8.11)

set(XP_3rdparty_path $ENV{HOME}/XP_release/3rdparty_lib_lean)

if (NOT IS_DIRECTORY ${XP_3rdparty_path})
 message(FATAL_ERROR "Cannot find 3rdparty_lib_lean at ${XP_3rdparty_path}")
 return()
endif()

# Find Eigen3
if (IS_DIRECTORY /usr/local/include/eigen3)
 set(Eigen_INCLUDE_DIR /usr/local/include/eigen3)
elseif (IS_DIRECTORY /usr/include/eigen3)
 set(Eigen_INCLUDE_DIR /usr/include/eigen3)
else()
 message(FATAL_ERROR "Cannot find Eigen3 at either /usr/local/include/eigen3 or /usr/include/eigen3")
 return()
endif()
message(STATUS "Found Eigen3 at Eigen_INCLUDE_DIR = ${Eigen_INCLUDE_DIR}")

# OpenCV_INCLUDE_DIRS and OpenCV_LIBS are set here
set(OpenCV_path ${XP_3rdparty_path})
find_package(OpenCV 3.0.0 REQUIRED
 PATHS ${OpenCV_path}
 NO_DEFAULT_PATH
)

if (NOT OpenCV_FOUND)
 message(FATAL_ERROR "Cannot find OpenCV")
 return()
else()
 message(STATUS "Found OpenCV ${OpenCV_VERSION} at ${OpenCV_path}")
 message(STATUS "OpenCV contains the following libs")
 foreach(lib_name ${OpenCV_LIBS})
  message(STATUS ${lib_name})
 endforeach()
endif()

find_path(JSON_INCLUDE_DIR json PATHS ${XP_3rdparty_path}/include/)
find_library(JSON_LIBRARY jsoncpp PATHS ${XP_3rdparty_path}/lib/)
