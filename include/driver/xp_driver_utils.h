/******************************************************************************
 * Copyright 2017-2019 Baidu Robotic Vision Authors. All Rights Reserved.
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
#ifndef INCLUDE_DRIVER_XP_DRIVER_UTILS_H_
#define INCLUDE_DRIVER_XP_DRIVER_UTILS_H_

#include <driver/base_sensor_driver.h>
#include <driver/XP_sensor_driver.h>
#include <driver/xp_driver_config.h>
#ifdef __RK_ENABLED__
#include <driver/rk_sensor_driver.h>
#endif
#include <string>

namespace XPDRIVER {
bool init_sensor(const std::string& sensor_type,
                 const bool auto_gain,
                 const bool imu_from_image,
                 const std::string& sensor_dev_path,
                 const std::string& sensor_dev_id,
                 const std::string& wb_mode,
                 std::unique_ptr<XPDRIVER::SensorMultithread>& sensor_ptr);  // NOLINT

}  // XPDRIVER
#endif  // INCLUDE_DRIVER_XP_DRIVER_UTILS_H_
