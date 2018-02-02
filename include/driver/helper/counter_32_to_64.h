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
#ifndef INCLUDE_DRIVER_HELPER_COUNTER_32_TO_64_H_
#define INCLUDE_DRIVER_HELPER_COUNTER_32_TO_64_H_

#include <stdint.h>

namespace XPDRIVER {

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

}  // namespace XPDRIVER
#endif  // INCLUDE_DRIVER_HELPER_COUNTER_32_TO_64_H_
