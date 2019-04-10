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
#ifndef INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_
#define INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_

/** [NOTE]
 * 1. XP sensor driver is the multi-thread implementation that drives the supported sensors
 *    (XP / XP2 / XP3) to get hardware synchronized images and IMU measurements
 * 2. XpSensorMultithread derives from base class SensorMultithread.
 * 3. There are two ways to get IMUs from supported sensors.
 *      a). Pull IMU measurements directly from the sensor
 *      b). Read IMU measurements embedded in the first row(s) of the image
 * 4. XP sensor driver only supports Linux for now.
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/basic_image_utils.h>  // For computeNewAecTableIndex
#include <driver/xp_sensors_wb_table.h>
#include <driver/base_sensor_driver.h>
#include <driver/XP_sensor.h>
#include <driver/v4l2.h>
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <driver/helper/ring_buffer.h>  // For RingBuffer
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define XP_CLOCK_32BIT_MAX_COUNT 0x00000000ffffffff
namespace XPDRIVER {

#ifndef __linux__
class XpSensorMultithread : public SensorMultithread {
 public:
  // Dummy implementation
  XpSensorMultithread(const std::string& sensor_type_str,
                      const bool use_auto_gain,
                      const bool imu_from_image,
                      const std::string& dev_name = "",
                      const std::string& dev_id = "",
                      const std::string& wb_mode = "auto") :
    SensorMultithread(sensor_type_str, use_auto_gain, imu_from_image, dev_name, dev_id, wb_mode) {}
  bool init() override { return false; }
  bool run() override { return false; }
  bool stop() override { return false; }
  bool get_sensor_deviceid(std::string* device_id) override { return false; }
  bool get_sensor_type(SensorType* sensor_type) override { return false; }
  bool store_calib_to_sensor(const std::string& calib_str) override { return false; }
  bool get_calib_from_sensor(std::string* calib_str) override { return false; }
};
#else
class XpSensorMultithread : public SensorMultithread {
 public:
  typedef std::pair<std::string, std::string> DevNameAndDevID;

  // Core functions
  XpSensorMultithread(const std::string& sensor_type_str,
                      const bool use_auto_gain,
                      const bool imu_from_image,
                      const std::string& dev_name = "",
                      const std::string& dev_id = "",
                      const std::string& wb_mode = "auto");
  ~XpSensorMultithread();
  bool init() override;
  bool run() override;
  bool stop() override;

  // Setters
  bool set_auto_gain(const bool use_aec) override;
  bool set_awb_mode(bool AutoMode, float coeff_r, float coeff_g, float coeff_b) override;
  bool set_ir_period(const int ir_period) override;
  bool set_key_control(const char keypressed) override;
  // Getters
  bool get_calib_from_sensor(std::string *calib_str) override;
  bool get_ir_on_status(void) override;
  bool get_sensor_deviceid(std::string* device_id) override;
  bool get_sensor_resolution(uint16_t* width, uint16_t* height) override;
  bool get_sensor_soft_ver(XpSoftVersion* soft_ver) override;
  bool get_sensor_type(SensorType* sensor_type) override;
  bool is_color() const override;
  bool store_calib_to_sensor(const std::string& calib_str) override;

 protected:
  // Queue and Dequeue ioctl buffer as soon as possible
  // If ioctl queue is not retrieved on time, it may crash Odroid
  // Not a big problem on PC
  void thread_ioctl_control() override;
  void thread_pull_imu() override;
  void thread_stream_images() override;

  void scan_videos(std::vector<DevNameAndDevID>* name_and_id);
  bool find_dev_name(const std::string& deviceID, std::string* dev_name_ptr);

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
  bool get_color_img_from_raw_data(const uint8_t* img_data_ptr,
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
  bool set_infrared_param(const XP_SENSOR::infrared_mode_t IR_mode, const int infrared_index,
                        const int ir_period);
  // set_aec_index only sets aec_index_
  // The AEC change will be applied to the sensor in thread_stream_images
  bool set_aec_index(const int aec_index);
  inline bool process_ir_control(char keypressed);
  inline bool process_gain_control(char keypressed);
  // Member variables for sensor control
  bool open_rgb_ir_mode_;
  bool use_auto_infrared_;
  int rgb_ir_period_;
  XP_SENSOR::infrared_mode_t ir_mode_;
  std::atomic<bool> ir_ctl_updated_;
  std::atomic<uint64_t> congested_ms_;
  uint8_t infrared_index_;
  struct v4l2_buffer bufferinfo_;
  std::unique_ptr<XPDRIVER::V4L2> xp_v4l2_ptr_;

  // push by thread_ioctl_control. Fetch by thread_stream_images
  XPDRIVER::shared_queue<RawPtrAndSysTime> raw_sensor_img_mmap_ptr_queue_;
  std::shared_ptr<AutoWhiteBalance> whiteBalanceCorrector_;
};

#endif  // __linux__

}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_
