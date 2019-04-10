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
#ifndef INCLUDE_DRIVER_RK_SENSOR_DRIVER_H_
#define INCLUDE_DRIVER_RK_SENSOR_DRIVER_H_

/** [NOTE]
 * 1. rk sensor driver is the multi-thread implementation that drives the supported sensor
 *    (boteyeOne) to get hardware synchronized images and IMU measurements.
 *    It derives from base sensor driver.
 * 2. There is only one way to get IMUs from sensor, i.e. Pull IMU measurements directly
 *    from the sensor
 * 3. rk sensor driver only supports Linux(Debian release version) for now.
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/counter_32_to_64.h>
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <driver/helper/sync_shared_queue.h>
#include <driver/base_sensor_driver.h>
#include <driver/rk_sensor.h>
#include <driver/xp_driver_config.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <unistd.h>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

namespace XPDRIVER {
#ifndef __RK_ENABLED__
class RkSensorMultithread : public SensorMultithread {
 public:
  RkSensorMultithread(const std::string& sensor_type_str = "",
                      const bool use_auto_gain = false,
                      const bool imu_from_image = false,
                      const std::string& dev_name = "",
                      const std::string& wb_mode = "") : \
    SensorMultithread(sensor_type_str, use_auto_gain, imu_from_image, dev_name, "", wb_mode) {
    std::cout << "[Error]: RK driver unsupported in this OS environment, "
              << "please do not call class RkSensorMultithread\n";
  }
  ~RkSensorMultithread();
  bool init() override { return false; }
  bool run() override { return false; }
  bool stop() override { return false; }

  // Redefined functions
  // Setters
  bool set_auto_gain(const bool use_aec) override { return false; }
  bool set_awb_mode(bool AutoMode, float coeff_r, float coeff_g, float coeff_b) override {
    return false;
  }
  bool set_sys_IR_callback(const SysImageDataCallback& callback) override { return false; }
  bool set_steady_IR_callback(const SteadyImageDataCallback& callback) override { return false; }
  bool set_key_control(const char keypressed) override { return false; }
  bool set_ir_period(const int ir_period) override { return false; }
  // Getters
  bool get_sensor_soft_ver(XpSoftVersion* soft_ver) override { return false; }
  bool get_sensor_resolution(uint16_t* width, uint16_t* height) override { return false; }
  bool get_sensor_deviceid(std::string* device_id) override { return false; }
  bool get_sensor_type(SensorType* sensor_type) override { return false; }
  bool get_calib_from_sensor(std::string *calib_str) override { return false; }
  bool store_calib_to_sensor(const std::string& calib_str) override { return false; }
  bool is_color() const override { return false; }
  bool get_ir_on_status(void) override { return false; }
};
#else
using XPDRIVER::RkSingleSensor;
class RkSensorMultithread : public SensorMultithread {
 public:
  // Core functions
  RkSensorMultithread(const std::string& sensor_type_str = "",
                      const bool use_auto_gain = false,
                      const bool imu_from_image = false,
                      const std::string& dev_name = "",
                      const std::string& wb_mode = "");
  ~RkSensorMultithread();
  bool init() override;
  bool run() override;
  bool stop() override;

  // Redefined functions
  // Setters
  bool set_auto_gain(const bool use_aec) override;
  bool set_awb_mode(bool AutoMode, float coeff_r, float coeff_g, float coeff_b) override;
  bool set_sys_IR_callback(const SysImageDataCallback& callback) override { return false; }
  bool set_steady_IR_callback(const SteadyImageDataCallback& callback) override { return false; }
  bool set_key_control(const char keypressed) override;
  bool set_ir_period(const int ir_period) override { return false; }
  // Getters
  bool get_sensor_soft_ver(XpSoftVersion* soft_ver) override { return false; }
  bool get_sensor_resolution(uint16_t* width, uint16_t* height) override;
  bool get_sensor_deviceid(std::string* device_id) override;
  bool get_sensor_type(SensorType* sensor_type) override;
  bool get_calib_from_sensor(std::string *calib_str) override;
  bool store_calib_to_sensor(const std::string& calib_str) override;
  bool is_color() const override;
  bool get_ir_on_status(void) override { return false; }

 protected:
  struct SingleImage {
    bool is_older_than(const SingleImage& other) const {
      uint64_t ts_ms_self = rk_timestamp.to_ms();
      uint64_t ts_ms_other = other.rk_timestamp.to_ms();
      if (ts_ms_self + kSyncToleranceMsec < ts_ms_other) {
        return true;
      }
      return false;
    }
    bool is_newer_than(const SingleImage& other) const {
      uint64_t ts_ms_self = rk_timestamp.to_ms();
      uint64_t ts_ms_other = other.rk_timestamp.to_ms();
      if (ts_ms_self > ts_ms_other + kSyncToleranceMsec) {
        return true;
      }
      return false;
    }
    cv::Mat data;
    RkTimestamp rk_timestamp;
    constexpr static uint64_t kSyncToleranceMsec = 5;  // 5 ms
  };
  typedef std::vector<SingleImage> SyncImages;

  struct inv_imu_data {
    int16_t accl_0;
    int16_t accl_1;
    int16_t accl_2;
    int16_t temp;
    int16_t gyro_0;
    int16_t gyro_1;
    int16_t gyro_2;
    int16_t dummy;
    // struct timeval tv;
    uint64_t tv_sec;
    uint64_t tv_usec;
  };

  // Queue and Dequeue ioctl buffer as soon as possible
  // If ioctl queue is not retrieved on time, it may crash rk sensor
  void thread_ioctl_control() override { return; }
  void thread_pull_imu() override;
  void thread_stream_images() override;
  void l_img_callback(const cv::Mat& img_rgb, const RkTimestamp rk_timestamp);
  void r_img_callback(const cv::Mat& img_rgb, const RkTimestamp rk_timestamp);

  // rk imu functions
  int write_sysfs_int(const char *filename, const char *basedir, int val, bool verify);
  int write_sysfs_int_no_verify(const char *filename, const char *basedir, int val);
  int write_sysfs_int_and_verify(const char *filename, const char *basedir, int val);
  int read_sysfs_posint(const char *filename, const char *basedir);
  int calc_digits(int num);
  int iio_find_type_by_name(const char *name, const char *type);
  bool imu_configure(uint8_t gfs, uint8_t afs, uint16_t samp_rate);
  int rk_imu_init(void);
  int rk_imu_deinit(void);
  void hex_dump(const unsigned char *buf, int len);
  bool IMU_data_access(inv_imu_data* inv_data_ptr, XP_20608_data* xp_20608_ptr);
  void convert_imu_axes(const XP_20608_data& imu_data, XPDRIVER::ImuData* xp_imu_ptr);
  void convert_rk_timestamp(const RkTimestamp& rk_timestamp, uint64_t* xp_timestamp);

  // Member variables for sensor control
  // The unique instance of RkSensorMultithread
  std::unique_ptr<XPDRIVER::RkSingleSensor> rk_sensor_l_ptr_;
  std::unique_ptr<XPDRIVER::RkSingleSensor> rk_sensor_r_ptr_;
  XPDRIVER::SyncSharedQueue<SingleImage> sync_shared_queue_;
  bool imu_verbose_;
  RkTimestamp first_imu_timestamp_;
  std::string cam_info_folder_;
};
#endif  // __RK_ENABLED__
}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_RK_SENSOR_DRIVER_H_
