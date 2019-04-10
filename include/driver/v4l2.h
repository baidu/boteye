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
#ifndef INCLUDE_DRIVER_V4L2_H_
#define INCLUDE_DRIVER_V4L2_H_
#include <stdint.h>
#include <string>
#include <vector>

#ifdef __linux__  // Only support linux for now
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <unistd.h>

namespace XPDRIVER {
// V4L2 related functions
class V4L2 {
 public:
  static const int V4L2_BUFFER_NUM;
  explicit V4L2(const std::string& dev_name);
  V4L2(const V4L2 &rhs) = delete;
  V4L2 & operator=(const V4L2 & rhs) = delete;

  bool init_mmap(int fd);
  virtual bool init_v4l2(const std::string& dev_name,
                        int* fd_ptr,
                        struct v4l2_buffer* bufferinfo_ptr);
  virtual bool stop_v4l2(const struct v4l2_buffer& bufferinfo);
  // This will advance bufferinfo.index and queue next
  // no matter success or not
  // Some possile fail reasons: CPU load is too high and thus ptr=0xffffff
  bool access_next_img_and_queue_next(struct v4l2_buffer* bufferinfo_ptr,
                                      uint8_t ** img_data_ptr);
  bool access_next_img_pair_data(struct v4l2_buffer* bufferinfo_ptr,
                                 uint8_t ** img_data_ptr);
  bool queue_next_img_buffer(struct v4l2_buffer* bufferinfo_ptr);
  bool get_v4l2_resolution(int* width, int* height);

  int fd_;
  // V4L2 related utility functions implementation
  /* config the buffer */
  /* local structure */
 protected:
  struct Buffer {
    void *start;
    __u32 offset;
    size_t length;
  };
  std::string dev_name_;
  int v4l2_width_, v4l2_height_;
  std::vector<struct Buffer> mmap_buffers_;
};
}  // namespace XPDRIVER

#endif  // __linux__
#endif  // INCLUDE_DRIVER_V4L2_H_
