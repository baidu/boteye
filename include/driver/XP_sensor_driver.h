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
#ifndef INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_
#define INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_

/** [NOTE]
 * 1. XP sensor driver is the multi-thread implementation that drives the supported sensors
 *    (XP / XP2 / XP3) to get hardware synchronized images and IMU measurements
 * 2. There are two ways to get IMUs from supported sensors.
 *      a). Pull IMU measurements directly from the sensor
 *      b). Read IMU measurements embedded in the first row(s) of the image
 * 3. XP sensor driver only supports Linux for now.
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/basic_image_utils.h>  // For computeNewAecTableIndex
#include <driver/xp_sensors_wb_table.h>
#include <driver/XP_sensor.h>
#include <driver/v4l2.h>
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <driver/helper/ring_buffer.h>  // For RingBuffer
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace XPDRIVER {

#ifndef __linux__
class XpSensorMultithread {
 public:
  // Dummy implementation
  bool run() { return false; }
  bool stop() { return false; }
  bool get_sensor_deviceid(std::string* device_id) { return false; }
};
#else
class XpSensorMultithread {
 public:
  struct SensorResolution {
    uint16_t RowNum;
    uint16_t ColNum;
  };
  typedef std::function<
      void(const cv::Mat&, const cv::Mat&, const float,
           const std::chrono::time_point<std::chrono::steady_clock>)> ImageDataCallback;
  typedef std::function<void(const XPDRIVER::ImuData&)> ImuDataCallback;
  typedef std::pair<uint64_t, std::chrono::time_point<std::chrono::steady_clock>>
      TimestampAndSysTime;

  // Core functions
  XpSensorMultithread(const std::string& sensor_type_str,
                      const bool use_auto_gain,
                      const bool imu_from_image,
                      const std::string& dev_name = "",
                      const std::string& wb_mode = "auto");
  ~XpSensorMultithread();
  bool init();
  bool run();
  bool stop();

  // Setters
  // set_aec_index only sets aec_index_
  // The AEC change will be applied to the sensor in thread_stream_images
  bool set_aec_index(const int aec_index);
  bool set_auto_gain(const bool use_aec);
  bool set_infrared_param(const XP_SENSOR::infrared_mode_t IR_mode, const int infrared_index,
                          const int ir_period);
  bool set_image_data_callback(const ImageDataCallback& callback);
  bool set_IR_data_callback(const ImageDataCallback& callback);
  bool set_imu_data_callback(const ImuDataCallback& callback);
  // Getters
  float get_image_rate() const { return stream_images_rate_; }
  float get_imu_rate() const { return pull_imu_rate_; }
  float get_ir_image_rate() const { return stream_ir_images_rate_; }
  bool get_sensor_soft_ver(XpSoftVersion* soft_ver);
  bool get_sensor_resolution(uint16_t* width, uint16_t* height);
  bool get_sensor_deviceid(std::string* device_id);
  bool get_sensor_type(SensorType* sensor_type);
  bool is_color() const;

 protected:
  // Queue and Dequeue ioctl buffer as soon as possible
  // If ioctl queue is not retrieved on time, it may crash Odroid
  // Not a big problem on PC
  void thread_ioctl_control();
  void thread_pull_imu();
  void thread_stream_images();

  // [NOTE] The returned cv::Mat is CV_8UC1 if the sensor is mono-color,
  //        and CV_8UC3 if the sensor is color
  bool get_images_from_raw_data(const uint8_t* img_data_ptr,
                                cv::Mat* img_l_ptr,
                                cv::Mat* img_r_ptr,
                                cv::Mat* img_l_IR_ptr,
                                cv::Mat* img_r_IR_ptr);
  bool get_XPIRL2_img_from_raw_data(const uint8_t* img_data_ptr,
                                    cv::Mat* img_l_ptr,
                                    cv::Mat* img_r_ptr,
                                    cv::Mat* img_l_IR_ptr,
                                    cv::Mat* img_r_IR_ptr);
#ifdef __ARM_NEON__
  bool get_XPIRL2_img_from_raw_data_neon(const uint8_t* img_data_ptr,
                                    cv::Mat* img_l_ptr,
                                    cv::Mat* img_r_ptr,
                                    cv::Mat* img_l_IR_ptr,
                                    cv::Mat* img_r_IR_ptr);
#endif  // __ARM_NEON__
  bool get_v024_img_from_raw_data(const uint8_t* img_data_ptr,
                                 cv::Mat* img_l_ptr,
                                 cv::Mat* img_r_ptr);
  bool get_v034_img_from_raw_data(const uint8_t* img_data_ptr,
                                 cv::Mat* img_l_ptr,
                                 cv::Mat* img_r_ptr);
  void handle_col_shift_case(uint8_t* img_data_ptr, int col_shift);
  bool zero_col_shift_detect(const uint8_t* img_data_ptr,
                             int* xp_shift_num);
  bool sensor_MT9V_image_separate(const uint8_t* img_data_ptr,
                                  cv::Mat* img_l_ptr,
                                  cv::Mat* img_r_ptr);
  void convert_imu_axes(const XP_20608_data& imu_data,
                        const SensorType sensor_type,
                        XPDRIVER::ImuData* xp_imu_ptr) const;
  bool timestamp_check(const uint64_t ts_with_overflow,
                       const std::chrono::time_point<std::chrono::steady_clock>& sys_time);

  // Member variables for sensor control
  std::string sensor_type_str_;
  std::string wb_mode_str_;
  SensorType sensor_type_;
  std::string dev_name_;
  std::atomic<bool> is_running_;
  bool imu_from_image_;
  bool open_rgb_ir_mode_;
  int rgb_ir_period_;
  XP_SENSOR::infrared_mode_t ir_mode_;
  bool use_auto_gain_;
  std::atomic<bool> aec_index_updated_;
  int aec_index_;  // use signed int as the index can go to negative during calculation
  bool aec_settle_;
  bool use_auto_infrared_;
  std::atomic<bool> ir_ctl_updated_;
  std::atomic<uint64_t> congested_ms_;
  uint8_t infrared_index_;
  int video_sensor_file_id_;
  int imaging_FPS_;
  struct v4l2_buffer bufferinfo_;
  uint64_t first_imu_clock_count_ = 0;
  XP_SENSOR::XPSensorSpec XP_sensor_spec_;
  RingBuffer<TimestampAndSysTime> ts_ring_buffer_;
  uint64_t latest_img_ts_with_overflow_;

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

  // push by thread_ioctl_control. Fetch by thread_stream_images
  typedef std::pair<uint8_t*, std::chrono::time_point<std::chrono::steady_clock>> RawPtrAndSysTime;
  XPDRIVER::shared_queue<RawPtrAndSysTime> raw_sensor_img_mmap_ptr_queue_;

  // For callback functions
  ImageDataCallback image_data_callback_;
  ImageDataCallback IR_data_callback_;
  ImuDataCallback imu_data_callback_;
  std::shared_ptr<AutoWhiteBalance> whiteBalanceCorrector_;
};

#endif  // __linux__

}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_
