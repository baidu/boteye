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
#ifndef INCLUDE_DRIVER_V4L2_RK_H_
#define INCLUDE_DRIVER_V4L2_RK_H_

#include <driver/basic_datatype.h>
#include <driver/rkisp_interface.h>
#include <driver/v4l2.h>
#include <driver/xp_driver_config.h>
#ifdef __RK_ENABLED__
#include <string>

namespace XPDRIVER {
class V4L2_RK : public V4L2 {
 public:
  V4L2_RK(const std::string& dev_name,
          const std::string& isp_dev_name);
  V4L2_RK(const V4L2_RK &rhs) = delete;
  V4L2_RK & operator=(const V4L2_RK & rhs) = delete;

  // redefined functions
  bool init_v4l2(const std::string& dev_name,
                int* fd_ptr,
                struct v4l2_buffer* bufferinfo_ptr);
  bool stop_v4l2(const struct v4l2_buffer& bufferinfo);

  // new functions
  bool access_next_img_and_queue_next(struct v4l2_buffer* bufferinfo_ptr,
                                      uint8_t ** img_data_ptr,
                                      RkTimestamp* rk_timestamp_ptr);
  bool access_next_img_pair_data(struct v4l2_buffer* bufferinfo_ptr,
                                 uint8_t ** img_data_ptr,
                                 RkTimestamp* rk_timestamp_ptr);
  bool set_aec_mode(bool on);
  bool set_awb_mode(bool on);

 protected:
  static int rk_isp_start_cnt_;
  static const std::string rk_iq_file_;
  std::string rk_isp_dev_;
  void* rkisp_engine_;
};
}  // namespace XPDRIVER
#endif  // __RK_ENABLED__
#endif  // INCLUDE_DRIVER_V4L2_RK_H_

