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
/// \file
#ifndef PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
#define PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
// XP Sensor driver
#include <simple-web-server/server_http.hpp>
#include <driver/xp_driver_config.h>
#include <driver/xp_driver_utils.h>
#include <XP/helper/param.h>
// Parsing flags and logging
#include <gflags/gflags.h>
#include <glog/logging.h>
#ifdef HAS_ROS
#include <app_tracking/XpDriverImage.h>
#include <app_tracking/XpDriverImu.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#endif
#include <memory>  // unique_ptr
#include <string>
#include <chrono>

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
    XP_SENSOR = 0,
    ROS_SUBSCRIBER = 1,
    RECORD_LOADER = 2,
    HTTP_SENSOR = 3,
    NONE = 4
  };

  InterfaceType interface_type_ = NONE;

  std::unique_ptr<XPDRIVER::SensorMultithread> xp_sensor_;

  HttpServer http_server_;

  std::string video_dev_file_;
  std::thread record_loader_thread_;  // Do NOT included in thread_pools_
  std::atomic<bool> is_loader_running_;

  std::chrono::time_point<std::chrono::steady_clock> thread_stream_images_pre_timestamp_;
  std::chrono::time_point<std::chrono::steady_clock> thread_pull_imu_pre_timestamp_;
  std::atomic<int> stream_images_count_;
  std::atomic<float> stream_images_rate_;
  std::atomic<int> pull_imu_count_;
  std::atomic<float> pull_imu_rate_;

#ifdef HAS_ROS
  ros::NodeHandle nh_;
  ros::Publisher image_pub;
  ros::Publisher IMU_pub;
  uint32_t img_seq_ID_ = 0;
  uint32_t imu_seq_ID_ = 0;
#endif

  XpDriverInterface() : xp_sensor_(nullptr), g_img_l_ptr(nullptr) {
    stream_images_rate_ = 0;
    pull_imu_rate_ = 0;
    g_img_l_ptr.reset(new cv::Mat());
#ifdef HAS_ROS
    if (!ros::master::check()) {
      LOG(FATAL) << "ROS master has not started yet. "
                    "Please start roscore and restart the program.";
    }
    nh_ = ros::NodeHandle("boteye");
    image_pub = nh_.advertise<app_tracking::XpDriverImage>("sensor/image", 125);
    IMU_pub = nh_.advertise<app_tracking::XpDriverImu>("sensor/IMU_data", 500);
#endif
  }

  inline void calculate_img_imu_rate();

 public:
  /**
   * \brief Initialize the sensor to run with a live sensor.  The user is responsible to properly
   *        initilize the live sensor, register the data callbacks, and start/stop running.
   * \return success or not
   */
  bool init_XP_sensor(const std::string &sensor_type,
                      const bool auto_gain,
                      const bool imu_from_image,
                      const std::string sensor_dev_path,
                      const std::string sensor_dev_id,
                      const std::string wb_mode);
#ifdef HAS_ROS
  void imageRosCb(const app_tracking::XpDriverImagePtr &msg);
  void imuRosCb(const app_tracking::XpDriverImuPtr &msg);
#endif
  bool init_ros_subscriber();

  /**
   * \brief Initialzie the data loader that loads images and imu from a folder.
   *        The data loader will disable all kinds of live sensor
   * \param folder_path where to load the data (similar to record_path)
   * \return success or not
   */
  bool init_data_loader(const std::string &folder_path);
  void thread_record_loader(const std::string &data_full_path);

  bool init_http_sensor();
  void run();
  void stop();
  std::shared_ptr<cv::Mat> g_img_l_ptr;  // [NOTE] g_img_l_ptr is NOT 100% thread-safe
  bool get_sensor_deviceid(std::string *device_id);
  void get_data_rate(float *img_rate, float *imu_rate);
  XpDriverInterface(XpDriverInterface const &) = delete;
  void operator=(XpDriverInterface const &) = delete;
  bool auto_calib_load(XP::DuoCalibParam *calib_param) const;
  bool load_calib_from_folder(const std::string &load_path, XP::DuoCalibParam *calib_param) const;
  bool load_calib_from_file(const std::string &calib_file, XP::DuoCalibParam *calib_param) const;
};
}  // namespace live
#endif  // PC_APPS_APP_TRACKING_XP_DRIVER_INTERFACE_H_
