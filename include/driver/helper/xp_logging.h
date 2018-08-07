/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef INCLUDE_DRIVER_HELPER_XP_LOGGING_H_
#define INCLUDE_DRIVER_HELPER_XP_LOGGING_H_
#include <cstdlib>

/**
 * This file defines all the debug macros of detailed logging.
 */

// __DRIVER_GLOG_DEBUG__ is configured in the CMakeLists.txt
// Requires glog when enabled
#ifdef __DRIVER_GLOG_DEBUG__

#include <glog/logging.h>
#define XP_LOG_INFO(msg) { LOG(INFO) << msg; }
#define XP_LOG_WARNING(msg) { LOG(WARNING) << msg; }
#define XP_LOG_ERROR(msg) { LOG(ERROR) << msg; }
#define XP_LOG_FATAL(msg) { LOG(FATAL) << msg; }
#define XP_VLOG(vlog_level, msg) { VLOG(vlog_level) << msg; }
#define XP_CHECK(val) { CHECK(val); }
#define XP_CHECK_EQ(val1, val2) { CHECK_EQ(val1, val2); }
#define XP_CHECK_GT(val1, val2) { CHECK_GT(val1, val2); }
#define XP_CHECK_LT(val1, val2) { CHECK_LT(val1, val2); }
#define XP_CHECK_GE(val1, val2) { CHECK_GE(val1, val2); }
#define XP_CHECK_LE(val1, val2) { CHECK_LE(val1, val2); }
#define XP_CHECK_NOTNULL(val) { CHECK_NOTNULL(val); }

#else

#include <assert.h>
#include <iostream>

#include <sstream>
// Disable all debug loggings

#ifdef __ANDROID__
  #include <android/log.h>

  #define XP_LOG_INFO(msg) \
  {   \
    std::ostringstream oss;\
    oss << msg; oss.flush();\
    __android_log_print(ANDROID_LOG_INFO, "boteyeR:",  oss.str().c_str());\
    oss.clear();\
  }
  #define XP_LOG_WARNING(msg) \
  {   \
    std::ostringstream oss;\
    oss << msg; oss.flush();\
    __android_log_print(ANDROID_LOG_WARN, "boteyeR:",  oss.str().c_str());\
    oss.clear();\
  }
  #define XP_LOG_ERROR(msg) \
  {   \
    std::ostringstream oss;\
    oss << msg; oss.flush();\
    __android_log_print(ANDROID_LOG_ERROR, "boteyeR:",  oss.str().c_str());\
    oss.clear();\
  }
  #define XP_LOG_FATAL(msg) \
  {   \
    std::ostringstream oss;\
    oss << msg; oss.flush();\
    __android_log_print(ANDROID_LOG_ERROR, "boteyeR:",  oss.str().c_str());\
    oss.clear();\
    exit(EXIT_FAILURE);\
  }
//    #define XP_VLOG(vlog_level, msg)
  #define XP_VLOG(level, msg)
#else
#define XP_LOG_INFO(msg) { std::cout << msg << std::endl; }
#define XP_LOG_WARNING(msg) { std::cout << "WARNING: " << msg << std::endl; }
#define XP_LOG_ERROR(msg) { std::cerr << msg << std::endl; }
#define XP_LOG_FATAL(msg) { std::cerr << msg << std::endl; exit(EXIT_FAILURE);}
// TODO(zhoury) implement XP_VLOG
#define XP_VLOG(vlog_level, msg)
#endif

#define XP_CHECK(val) { assert(val); }
#define XP_CHECK_EQ(val1, val2) { assert((val1) == (val2)); }
#define XP_CHECK_GT(val1, val2) { assert((val1) > (val2)); }
#define XP_CHECK_LT(val1, val2) { assert((val1) < (val2)); }
#define XP_CHECK_GE(val1, val2) { assert((val1) >= (val2)); }
#define XP_CHECK_LE(val1, val2) { assert((val1) <= (val2)); }
#define XP_CHECK_NOTNULL(val) { assert(val); }

#endif  // __DRIVER_GLOG_DEBUG__
#endif  // INCLUDE_DRIVER_HELPER_XP_LOGGING_H_
