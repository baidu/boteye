# ----------------------------------------------------------------------------
# This cmake helps to locate the 3rdparty dependencies installed into system
# via via apt-get, including:
# Boost, GFlags, Glog (Optional: Curl, Freetype)
# ----------------------------------------------------------------------------

find_package(Boost
 COMPONENTS filesystem system thread
 REQUIRED
)

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  #installed via sudo apt-get install libgflags-dev
  find_library(GFLAGS_LIBRARY gflags PATHS /usr/lib/x86_64-linux-gnu/)
  #installed via sudo apt-get install libgoogle-glog-dev
  find_library(GLOG_LIBRARY glog PATHS /usr/lib/x86_64-linux-gnu/)
  if(EnableRecognition)
    find_library(CURL_LIBRARY curl PATHS /usr/lib/x86_64-linux-gnu/)
    find_path(FREETYPE_INCLUDE_DIR freetype2 PATHS /usr/include/)
    find_library(FREETYPE_LIBRARY freetype PATHS /usr/lib/x86_64-linux-gnu/)
  endif()
elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l")
  #installed via sudo apt-get install libgflags-dev
  find_library(GFLAGS_LIBRARY gflags PATHS /usr/lib/arm-linux-gnueabihf/)
  #installed via sudo apt-get install libgoogle-glog-dev
  find_library(GLOG_LIBRARY glog PATHS /usr/lib/arm-linux-gnueabihf/)
  if(EnableRecognition)
    find_library(CURL_LIBRARY curl PATHS /usr/lib/arm-linux-gnueabihf/)
    find_path(FREETYPE_INCLUDE_DIR freetype2 PATHS /usr/include/)
    find_library(FREETYPE_LIBRARY freetype PATHS /usr/lib/arm-linux-gnueabihf/)
  endif()
elseif (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  #installed via sudo apt-get install libgflags-dev
  find_library(GFLAGS_LIBRARY gflags PATHS /usr/lib/aarch64-linux-gnu/)
  #installed via sudo apt-get install libgoogle-glog-dev
  find_library(GLOG_LIBRARY glog PATHS /usr/lib/aarch64-linux-gnu/)
  if(EnableRecognition)
    find_library(CURL_LIBRARY curl PATHS /usr/lib/aarch64-linux-gnu/)
    find_path(FREETYPE_INCLUDE_DIR freetype2 PATHS /usr/include/)
    find_library(FREETYPE_LIBRARY freetype PATHS /usr/lib/aarch64-linux-gnu/)
  endif()
endif()
