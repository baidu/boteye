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
/// \file
#ifndef PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
#define PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
// XP Sensor driver
#include <XP/helper/server_http.hpp>
#include <driver/XP_sensor_driver.h>
#include <XP/helper/param.h>
#include <memory>  // unique_ptr
#include <string>

using HttpServer = SimpleWeb::Server<SimpleWeb::HTTP>;

namespace live {
class XpDriverInterface {
 public:
  static XpDriverInterface &getInstance() {
    static XpDriverInterface instance;
    return instance;
  }

 private:
  enum InterfaceType {
    XP_sensor = 0,
    rk_sensor = 1,
    http_sensor = 2,
    none = 3
  };

  InterfaceType interface_type_ = none;

  std::unique_ptr<XPDRIVER::XpSensorMultithread> xp_sensor_;

  HttpServer http_server_;
  std::chrono::time_point<std::chrono::steady_clock> thread_stream_images_pre_timestamp_;
  std::atomic<int> stream_images_count_;
  std::atomic<float> stream_images_rate_;
  std::atomic<int> pull_imu_count_;
  std::atomic<float> pull_imu_rate_;

  XpDriverInterface() : xp_sensor_(nullptr), g_img_l_ptr(nullptr) {
    stream_images_rate_ = 0;
    pull_imu_rate_ = 0;
    g_img_l_ptr.reset(new cv::Mat());
  }

 public:
  bool init_XP_sensor(const std::string &sensor_type,
                      const bool auto_gain,
                      const bool imu_from_image,
                      const std::string sensor_dev_path,
                      const std::string wb_mode);
  bool init_rk_sensor();
  bool init_http_sensor();
  void run();
  void stop();
  std::shared_ptr<cv::Mat> g_img_l_ptr;  // [NOTE] g_img_l_ptr is NOT 100% thread-safe
  bool get_sensor_deviceid(std::string *device_id);
  void get_data_rate(float *img_rate, float *imu_rate);
  bool register_data_callbacks();
  XpDriverInterface(XpDriverInterface const &) = delete;
  void operator=(XpDriverInterface const &) = delete;
  bool auto_calib_load(XP::DuoCalibParam* calib_param) const;
};
}  // namespace live
#endif  // PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
