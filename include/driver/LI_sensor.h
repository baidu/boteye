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
#ifndef INCLUDE_DRIVER_LI_SENSOR_H_
#define INCLUDE_DRIVER_LI_SENSOR_H_

// [NOTE] For now, LI sensor only works for Linux
#include <driver/basic_datatype.h>  // for XP_20608_data
#include <string>
#include <vector>
#ifndef LI_BOARD_CLOCK_HZ
#define LI_BOARD_CLOCK_HZ (25.1875e6)
#define LI_BOARD_CLOCK_HZ_INT 25187500
#endif

// LI clock overflows past a magic number, not 0xFFFF
// The max 32 bit clock value that is observed is 2999799031
// So the overflow magic numer is guessed to be the following val
#define LI_CLOCK_32BIT_MAX_COUNT 3000000000

namespace XPDRIVER {
namespace LI_SENSOR {

#ifdef __linux__  // predefined by gcc

bool IMU_DataAccess(int fd, XP_20608_data* data_ptr);

// write the sensor register value
bool SensorRegWrite(int fd, int regAddr, int regVal);
// read the sensor register value
int SensorRegRead(int fd, int regAddr);
#endif  // __linux__

}  // namespace LI_SENSOR
}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_LI_SENSOR_H_
