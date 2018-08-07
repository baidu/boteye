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
#ifndef INCLUDE_DRIVER_BASIC_DATATYPE_H_
#define INCLUDE_DRIVER_BASIC_DATATYPE_H_

#include <stdint.h>
#include <string>

namespace XPDRIVER {

struct ImuData {
  float time_stamp;
  float accel[3] {};
  float ang_v[3] {};
};

struct XP_20608_data {
  uint64_t clock_count;
  float accel[3];
  float gyro[3];
  float temp;
};
enum class ImageType {
  RGB = 1,
  IR = 2,
  Unkown_type = 255
};

enum class SensorType {
  XP = 0,
  XP2 = 1,
  XP3 = 2,
  FACE = 3,
  XPIRL = 4,
  XPIRL2 = 5,
  XPIRL3 = 6,
  Unkown_sensor = 255
};

const std::string SensorName[] = {"XP", "XP2", "XP3", "FACE", "XPIRL", "XPIRL2", "XPIRL3"};

// need to know camera len more info to refactor this structure
enum class CameraLenType {
  FOV_80  = 0,
  FOV_120 = 1
};


enum class LightSource {
  Fluorescent = 0,  // use this option for indoor envirioment
  Sunlight_morning = 1,  // outdoor
  Sunlight_noon = 2,  // outdoor
  Sunlight_twilight = 3,  // outdoor
};

struct XP_white_balance_params {
  SensorType sensor_type_;
  CameraLenType camera_len_type_;
  LightSource light_source_;
};
}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_BASIC_DATATYPE_H_
