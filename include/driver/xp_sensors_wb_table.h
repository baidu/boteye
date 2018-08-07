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

#ifndef INCLUDE_DRIVER_XP_SENSORS_WB_TABLE_H_
#define INCLUDE_DRIVER_XP_SENSORS_WB_TABLE_H_

namespace XPDRIVER {
namespace XP_SENSOR {
struct WB_PARAM {
  float r_;
  float g_;
  float b_;
};
// This table is lookuped by SensorType, when there is a new color sensor supported,
// we must sync up the white balance parameters
// TODO(yanghongtian): move this lookup table to SensorType side-by-side
static const WB_PARAM s_wb_param[] = {
  {1.f, 1.f, 1.f},            // invalid for XP sensor
  {1.f, 1.f, 1.f},            // invalid for XP2 sensor
  {1.46582f, 1.f, 1.33606f},  // XP3
  {1.46582f, 1.f, 1.33606f},  // FACE
  {1.f, 1.f, 1.f},            // XPIRL
  {1.f, 1.f, 1.f},            // XPIRL2
  {1.f, 1.f, 1.f},            // XPIRL3
};

}  // namespace XP_SENSOR
}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_SENSORS_WB_TABLE_H_
