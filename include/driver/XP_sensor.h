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
#ifndef INCLUDE_DRIVER_XP_SENSOR_H_
#define INCLUDE_DRIVER_XP_SENSOR_H_

// [NOTE] For now, XP sensor only works for Linux
#ifdef __linux__  // predefined by gcc
#include <linux/videodev2.h>
#endif  // __linux__
#ifdef  __CYGWIN__
// shared memory
#include <windows.h>
#endif

#include <driver/helper/counter_32_to_64.h>
#include <driver/basic_datatype.h>  // for XP_20608_data
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <atomic>

#ifndef XP_BOARD_CLOCK_HZ
#define XP_BOARD_CLOCK_HZ (25.1875e6)
#define XP_BOARD_CLOCK_HZ_INT 25187500
#define XP_BOARD_GYRO_SCALE 2000
#define XP_BOARD_ACCEL_SCALE (2 * 9.799)
#endif

#define XP_CLOCK_32BIT_MAX_COUNT 0x00000000ffffffff

namespace XPDRIVER {
namespace XP_SENSOR {
// address overflow issue
struct XPSensorSpec {
  char   video_num[20];
  char   dev_id[100];
  int    RowNum;
  int    ColNum;
  SensorType sensor_type;
  char   Soft_ver[64];
};
#ifdef __linux__  // predefined by gcc
bool IMU_DataAccess(int fd, XP_20608_data* data_ptr);
bool set_registers_to_default(int fd,
                              SensorType sensor_type,
                              int aec_index,
                              bool verbose = false,
                              uint32_t* exp_ptr = nullptr,
                              uint32_t* gain_ptr = nullptr);
bool read_register(int fd, int16_t addr, int16_t* val);
bool set_register(int fd, int16_t addr, int16_t val);
bool set_aec_index(int fd, uint32_t aec_index, bool verbose = false);
bool set_exp_percentage(int fd, int16_t val, bool verbose = false);
bool set_gain_percentage(int fd, int16_t val, bool verbose = false);
int set_auto_exp_and_gain(int fd, bool ae, bool ag);
bool xp_imu_embed_img(int fd, bool enable);
struct XPSensorSpec get_XP_sensor_spec(int fd);
bool read_soft_version(int fd, char* soft_ver_ptr);
SensorType read_hard_version(int fd);
void read_deviceID(int fd, char* device_id);
struct tlc59116_ctl_t {
  uint8_t UpdateBit: 1;
  uint8_t dumpRegister: 1;
  uint8_t WriteRegister: 1;
  uint8_t SetCH_on_mode: 1;
  uint8_t SetCH_pwm_mode: 1;
  uint8_t : 3;
  uint8_t pwm_value;
  uint16_t channel_value;
};
enum infrared_mode_t {
  off = 0,
  on = 1,
  pwm = 2
};
const uint32_t infrared_pwm_max = 255;
bool xp_infrared_ctl(int fd, infrared_mode_t infrared_mode, uint16_t channel_value,
                     uint8_t pwm_value);
bool xp_tl59116_dump_register(int fd);
#endif  // __linux__
uint64_t get_timestamp_in_img(const uint8_t* data);
bool stamp_timestamp_in_img(uint8_t* data, uint64_t time);
#ifdef  __CYGWIN__
class SharedMemoryReader {
 public:
  struct ImageData {
    std::shared_ptr<std::vector<uint8_t>> data;
    ImageData() {
      data.reset(new std::vector<uint8_t>);
    }
  };
  explicit SharedMemoryReader(const std::string& file);
  ~SharedMemoryReader();
  // this is a blocking call
  bool read(ImageData* data);
 protected:
  LPCTSTR Buf_ptr_;
};
#endif

// [NOTE] Deprecated class
class OpencvVideoCap {
 public:
  explicit OpencvVideoCap(int vid);
  ~OpencvVideoCap();
  bool retrive(std::shared_ptr<std::vector<uint8_t>> data_ptr);
  static const int IMU_DATA_POS = 640 * (480 * 3 - 3) + 0;
  static const int W = 640;
  static const int H = 480;
  static bool copy_to_lr(cv::Mat* img_l_ptr, cv::Mat* img_r_ptr, uint8_t* data);
 protected:
  cv::VideoCapture cap_;
  cv::Mat frame_cache_;
};

class ImuReader {
 public:
  ImuReader();
  ~ImuReader();
#ifdef __linux__  // predefined by gcc
  // return true if imu_data is ready
  // return false if the char so far is not enough to get a complete imu_data
  bool read(int fd, XP_20608_data* imu_data_ptr);
#endif  // __linux__
  bool get_imu_from_img(const uint8_t* data,
                        XP_20608_data* imu_data_ptr,
                        const bool use_100us = false);
  int imu_rate() const;
  int imu_sample_count() const;
  uint64_t first_imu_clock_count() const;

 private:
  bool get_vec3f_from_sensor_data(const uint8_t* data, float* v);
  bool get_vec3f_from_img_data(const uint8_t* data, float* v);

 private:
  const float accel_scale_ = XP_BOARD_ACCEL_SCALE;
  const float gyro_scale_ = XP_BOARD_GYRO_SCALE;
  int idx_;
  uint8_t* buf_;
  std::unique_ptr<XPDRIVER::Counter32To64> counter32To64_ptr_;
  // to count IMU rate
  std::atomic<int> imu_rate_;
  std::atomic<int> imu_sample_count_;
  std::atomic<int> imu_sample_count_for_rate_;
  std::atomic<uint64_t> first_imu_clock_count_;
  std::chrono::time_point<std::chrono::steady_clock> imu_sample_start_tp_;
};
}  // namespace XP_SENSOR
}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_SENSOR_H_
