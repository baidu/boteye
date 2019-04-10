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
#ifndef INCLUDE_DRIVER_RK_SENSOR_H_
#define INCLUDE_DRIVER_RK_SENSOR_H_

/** [NOTE]
 * 1. RkSingleSensor is the multi-thread implementation that drives the rk sensor
 *    to get hardware synchronized images.
 * 2. RkSingleSensor only supports Linux(debian) aarch64 for now.
 */
#include <driver/basic_datatype.h>  // For ImuData & XP_20608_data
#include <driver/helper/shared_queue.h>  // For shared_queue
#include <driver/helper/xp_logging.h>
#include <driver/v4l2_rk.h>
#include <driver/xp_driver_config.h>
#ifdef __RK_ENABLED__
#include <opencv2/core.hpp>
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
#include <utility>  // For pair<>

namespace XPDRIVER {

class RkSingleSensor {
 public:
  typedef std::function<void(const cv::Mat&, RkTimestamp)> SensorDataCallback;
  // Core functions
  RkSingleSensor(const std::string& sensor_dev,
                 const std::string& isp_dev,
                 const std::string& sensor_name = "");
  ~RkSingleSensor();
  bool init();
  bool run();
  bool stop();

  // Setters
  bool set_sensor_data_callback(const SensorDataCallback& callback);

  // Getters
  float get_image_rate() const { return stream_images_rate_rk_; }
  bool get_resolution(uint16_t* width, uint16_t* height);
  bool set_auto_exposure(bool on);
  bool set_auto_white_balance(bool on);

 protected:
  void thread_ioctl_control_rk();
  void thread_stream_images_rk();
  int convert_nv12_to_rgb(unsigned char *nv12_ptr,
                          cv::Mat &rgb,  // NOLINT
                          unsigned int width,
                          unsigned int height);
  bool is_sysclock_legal(const RkTimestamp& pre_ts, const RkTimestamp& cur_ts);

  // Member variables for sensor control
  std::string sensor_dev_;
  std::string isp_dev_;
  std::string sensor_name_;
  std::atomic<bool> is_running_;
  int video_fd_rk_;
  int imaging_FPS_;
  int sensor_row_;
  int sensor_col_;
  int verbose_;
  // push by thread_ioctl_control. Fetch by thread_stream_images
  typedef std::pair<uint8_t*, RkTimestamp> RawPtrAndSysTime;
  XPDRIVER::shared_queue<RawPtrAndSysTime> raw_sensor_img_mmap_ptr_queue_;
  // For threading and timing stats
  std::vector<std::thread> thread_pool_;
  std::atomic<float> stream_images_rate_rk_;
  std::atomic<int> stream_images_count_rk_;
  std::chrono::time_point<std::chrono::steady_clock> thread_stream_pre_timestamp_;
  // For callback functions
  SensorDataCallback image_data_callback_;
  std::unique_ptr<XPDRIVER::V4L2_RK> rk_v4l2_ptr_;
  struct v4l2_buffer bufferinfo_;
};


}  // namespace XPDRIVER

#endif  // __RK_ENABLED__
#endif  // INCLUDE_DRIVER_RK_SENSOR_H_
