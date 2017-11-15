/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
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
#ifndef XP_INCLUDE_XP_DRIVER_LI_SENSOR_H_
#define XP_INCLUDE_XP_DRIVER_LI_SENSOR_H_

// [NOTE] For now, LI sensor only works for Linux
#ifdef __linux__  // predefined by gcc
#include <linux/videodev2.h>
#endif  // __linux__
#include <string>
#include <vector>
#ifndef LI_BOARD_CLOCK_HZ
#define LI_BOARD_CLOCK_HZ (25.1875e6)
#define LI_BOARD_CLOCK_HZ_INT 25187500
#endif
namespace XP_DRIVER {
namespace LI_SENSOR {
struct XP_20608_data {
  uint64_t clock_count;
  float accel[3];
  float gyro[3];
  float temp;
};

// LI clock overflows past a magic number, not 0xFFFF
// The max 32 bit clock value that is observed is 2999799031
// So the overflow magic numer is guessed to be the following val
#define LI_CLOCK_32BIT_MAX_COUNT 3000000000
#define LI_V4L2_BUFFER_NUM 6  // 8 makes Odroid init too slow
#ifdef __linux__  // predefined by gcc
bool init_mmap(int fd);
bool init_v4l2(const std::string& dev_name,
               int* fd_ptr,
               struct v4l2_buffer* bufferinfo_ptr);
bool stop_v4l2(int* fd_ptr, const struct v4l2_buffer& bufferinfo);
// This will increment bufferinfo.index and queue next
// no matter success or not
// Some possile fail reasons: CPU load is too high and thus ptr=0xffffff
bool access_next_img_and_queue_next(int fd,
                                    struct v4l2_buffer* bufferinfo_ptr,
                                    uint8_t ** img_data_ptr);
bool queue_next_img_buffer(int fd, struct v4l2_buffer* bufferinfo_ptr);
bool access_next_img_pair_data(int fd,
                               struct v4l2_buffer* bufferinfo_ptr,
                               uint8_t ** img_data_ptr);

bool IMU_DataAccess(int fd, XP_20608_data* data_ptr);

// write the sensor register value
bool SensorRegWrite(int fd, int regAddr, int regVal);
// read the sensor register value
int SensorRegRead(int fd, int regAddr);
#endif  // __linux__
// address overflow issue
class Counter32To64 {
 public:
  explicit Counter32To64(uint64_t max_clock_count);
 public:
  // we cannot use uint32_t because << 32 won't work for 32
  // bit number with x86, though the meaningful data
  // is only 32 bits
  uint64_t convertNewCount32(uint64_t counter32);
  // Not allowed. Will crash for debug purpose
  uint64_t convertNewCount32(uint32_t counter32);
  uint64_t convertNewCount32(int32_t counter32);
  uint64_t convertNewCount32(int64_t counter32);
  uint64_t getOverflowCount() const;
  uint64_t getLastCounter32() const;
 private:
  uint64_t overflow_count_ = 0;
  uint64_t last_counter32_ = 0;
  const uint64_t max_clock_count_ = 0;
};
}  // namespace LI_SENSOR
}  // namespace XP_DRIVER
#endif  // XP_INCLUDE_XP_DRIVER_LI_SENSOR_H_
