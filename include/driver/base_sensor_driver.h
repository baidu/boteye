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
#ifndef INCLUDE_DRIVER_BASE_SENSOR_DRIVER_H_
#define INCLUDE_DRIVER_BASE_SENSOR_DRIVER_H_

/**[NOTE]
 * base sensor driver defines the APIs for all kinds of supported sensors
 * (xp / rk / http sensor).
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/basic_image_utils.h>  // For computeNewAecTableIndex
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <driver/helper/ring_buffer.h>  // For RingBuffer
#include <driver/XP_sensor.h>
#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace XPDRIVER {

#ifndef __linux__
class SensorMultithread {
 public:
  // Dummy implementation
  SensorMultithread(const std::string& sensor_type_str,
                    const bool use_auto_gain,
                    const bool imu_from_image,
                    const std::string& dev_name = "",
                    const std::string& dev_id = "",
                    const std::string& wb_mode = "auto") {}
  virtual bool init() { return false; }
  virtual bool run() { return false; }
  virtual bool stop() { return false; }
  virtual bool get_sensor_deviceid(std::string* device_id) { return false; }
  virtual bool get_sensor_type(SensorType* sensor_type) { return false; }
  virtual bool store_calib_to_sensor(const std::string& calib_str) { return false; }
  virtual bool get_calib_from_sensor(std::string* calib_str) { return false; }
};
#else
class SensorMultithread {
 public:
  struct SensorResolution {
    uint16_t RowNum;
    uint16_t ColNum;
  };
  typedef std::function<
    void(const cv::Mat&, const cv::Mat&, const float,
          const std::chrono::time_point<std::chrono::system_clock>)> SysImageDataCallback;
  typedef std::function<
    void(const cv::Mat&, const cv::Mat&, const float,
          const std::chrono::time_point<std::chrono::steady_clock>)> SteadyImageDataCallback;
  typedef std::function<void(const XPDRIVER::ImuData&)> ImuDataCallback;
  typedef std::pair<uint64_t, std::chrono::time_point<std::chrono::steady_clock>>
      TimestampAndSysTime;

  // Core functions
  SensorMultithread(const std::string& sensor_type_str,
                    const bool use_auto_gain,
                    const bool imu_from_image,
                    const std::string& dev_name = "",
                    const std::string& dev_id = "",
                    const std::string& wb_mode = "auto");
  virtual ~SensorMultithread();

  // pure virtual functions
  virtual bool init() = 0;
  virtual bool run() = 0;
  virtual bool stop() = 0;
  // Setters
  virtual bool set_auto_gain(const bool use_aec) = 0;
  virtual bool set_awb_mode(bool AutoMode, float coeff_r, float coeff_g, float coeff_b) = 0;
  virtual bool set_ir_period(const int ir_period) = 0;
  virtual bool set_key_control(const char keypressed) = 0;
  // Getters
  virtual bool get_calib_from_sensor(std::string *calib_str) = 0;
  virtual bool get_ir_on_status(void) = 0;
  virtual bool get_sensor_deviceid(std::string* device_id) = 0;
  virtual bool get_sensor_resolution(uint16_t* width, uint16_t* height) = 0;
  virtual bool get_sensor_soft_ver(XpSoftVersion* soft_ver) = 0;
  virtual bool get_sensor_type(SensorType* sensor_type) = 0;
  virtual bool is_color() const = 0;
  virtual bool store_calib_to_sensor(const std::string& calib_str) = 0;

  // virtual functions
  virtual bool is_ir() const { return is_ir_sensor_; }
  virtual bool set_imu_data_callback(const ImuDataCallback& callback);
  virtual bool set_steady_image_callback(const SteadyImageDataCallback& callback);
  virtual bool set_steady_IR_callback(const SteadyImageDataCallback& callback);
  virtual bool set_sys_image_callback(const SysImageDataCallback& callback);
  virtual bool set_sys_IR_callback(const SysImageDataCallback& callback);

  // normal functions
  uint64_t get_current_frame_index() const { return frame_counter_; }
  float get_image_rate() const { return stream_images_rate_; }
  float get_imu_rate() const { return pull_imu_rate_; }
  float get_ir_image_rate() const { return stream_ir_images_rate_; }

 protected:
  virtual void thread_ioctl_control() = 0;
  virtual void thread_pull_imu() = 0;
  virtual void thread_stream_images() = 0;

  // Member variables for sensor control
  std::string dev_id_str_;
  std::string dev_name_;
  std::string sensor_type_str_;
  std::string wb_mode_str_;
  SensorType sensor_type_;
  std::atomic<bool> aec_index_updated_;
  std::atomic<bool> is_running_;
  int aec_index_;  // use signed int as the index can go to negative during calculation
  int imaging_FPS_;
  int video_fd_;
  uint64_t first_imu_clock_count_ = 0;
  uint64_t frame_counter_;
  uint64_t latest_img_ts_with_overflow_;
  bool aec_settle_;
  bool imu_from_image_;
  bool is_ir_sensor_;
  bool use_auto_gain_;
  XP_SENSOR::XPSensorSpec XP_sensor_spec_;
  RingBuffer<TimestampAndSysTime> ts_ring_buffer_;

  // For threading and timing stats
  std::vector<std::thread> thread_pool_;
  std::atomic<float> stream_images_rate_;
  std::atomic<int> stream_images_count_;
  std::atomic<float> stream_ir_images_rate_;
  std::atomic<int> stream_ir_images_count_;
  std::chrono::time_point<std::chrono::steady_clock> thread_stream_images_pre_timestamp_;
  std::atomic<float> pull_imu_rate_;
  std::atomic<int> pull_imu_count_;
  std::chrono::time_point<std::chrono::steady_clock> thread_pull_imu_pre_timestamp_;

  typedef std::pair<uint8_t*, std::chrono::time_point<std::chrono::steady_clock>> RawPtrAndSysTime;

  // For callback functions
  SysImageDataCallback image_callback_with_sys_clock_;
  SteadyImageDataCallback image_callback_with_steady_clock_;
  SysImageDataCallback IR_callback_with_sys_clock_;
  SteadyImageDataCallback IR_callback_with_steady_clock_;
  ImuDataCallback imu_data_callback_;
};

#endif  // __linux__

}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_BASE_SENSOR_DRIVER_H_
