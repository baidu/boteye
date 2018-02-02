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
 *    (LI / XP / XP2 / XP3) to get hardware synchronized images and IMU measurements
 * 2. There are two ways to get IMUs from supported sensors.
 *      a). Pull IMU measurements directly from the sensor
 *      b). Read IMU measurements embedded in the first row(s) of the image
 * 3. XP sensor driver only supports Linux for now.
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/basic_image_utils.h>  // For computeNewAecTableIndex
#include <driver/XP_sensor.h>
#include <driver/LI_sensor.h>
#include <driver/v4l2.h>
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace XPDRIVER {

#ifndef __linux__
class XpSensorMultithread {
};
#else
class XpSensorMultithread {
 public:
  struct SensorResolution {
    uint16_t RowNum;
    uint16_t ColNum;
  };
  typedef std::function<void(const cv::Mat&, const cv::Mat&, const float)> ImageDataCallback;
  typedef std::function<void(const XPDRIVER::ImuData&)> ImuDataCallback;

  // Core functions
  XpSensorMultithread(const std::string& sensor_type_str,
                      const bool use_auto_gain,
                      const bool imu_from_image,
                      const std::string& dev_name = "",
                      const std::string& wb_mode = "auto");
  ~XpSensorMultithread();
  bool init(const int aec_index);
  bool run();
  bool stop();

  // Setters
  // set_aec_index only sets aec_index_
  // The AEC change will be applied to the sensor in thread_stream_images
  bool set_aec_index(const int aec_index);
  bool set_auto_gain(const bool use_aec);
  bool set_auto_infrared(const bool use_infrared);
  bool set_infrared_index(const int infrared_index);
  bool set_image_data_callback(const ImageDataCallback& callback);
  bool set_imu_data_callback(const ImuDataCallback& callback);

  // Getters
  float get_image_rate() const { return stream_images_rate_; }
  float get_imu_rate() const { return pull_imu_rate_; }
  bool get_sensor_resolution(uint16_t* width, uint16_t* height);
  bool get_sensor_deviceid(std::string* device_id);
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
                                cv::Mat* img_l,
                                cv::Mat* img_r);
  void convert_imu_axes(const XP_20608_data& imu_data,
                        const SensorType sensor_type,
                        XPDRIVER::ImuData* xp_imu_ptr) const;

  // Member variables for sensor control
  std::string sensor_type_str_;
  std::string wb_mode_str_;
  std::string sensor_device_id_;
  SensorType sensor_type_;
  SensorResolution sensor_resolution_;
  std::string dev_name_;
  std::atomic<bool> is_running_;
  bool imu_from_image_;
  bool use_auto_gain_;
  std::atomic<bool> aec_index_updated_;
  int aec_index_;  // use signed int as the index can go to negative during calculation
  bool use_auto_infrared_;
  std::atomic<bool> infrared_index_updated_;
  uint8_t infrared_index_;
  int video_sensor_file_id_;
  int imaging_FPS_;
  struct v4l2_buffer bufferinfo_;
  uint64_t first_imu_clock_count_ = 0;

  // For threading and timing stats
  std::vector<std::thread> thread_pool_;
  std::atomic<float> stream_images_rate_;
  std::atomic<int> stream_images_count_;
  std::chrono::time_point<std::chrono::steady_clock> thread_stream_images_pre_timestamp_;
  std::atomic<float> pull_imu_rate_;
  std::atomic<int> pull_imu_count_;
  std::chrono::time_point<std::chrono::steady_clock> thread_pull_imu_pre_timestamp_;
  // push by thread_ioctl_control. Fetch by thread_stream_images
  XPDRIVER::shared_queue<uint8_t*> raw_sensor_img_mmap_ptr_queue_;

  // For callback functions
  ImageDataCallback image_data_callback_;
  ImuDataCallback imu_data_callback_;
  std::shared_ptr<AutoWhiteBalance> whiteBalanceCorrector_;
};

#endif  // __linux__

}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_XP_SENSOR_DRIVER_H_
