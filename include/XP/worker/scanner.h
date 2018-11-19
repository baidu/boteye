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

#ifndef XP_INCLUDE_XP_WORKER_SCANNER_H_
#define XP_INCLUDE_XP_WORKER_SCANNER_H_

#include <navigation/navigation_type.h>

class Scanner {
 public:
  Scanner() :
    is_initialzed_(false),
    is_scan_started_(false),
    valid_data_count_(0),
    min_angle_rad_(-1.0),
    max_angle_rad_(-1.0) {
  }

  virtual ~Scanner() {}

  virtual bool init() = 0;
  virtual bool isInitialized() { return is_initialzed_; }
  virtual bool isScanStarted() { return is_scan_started_; }
  virtual bool startScan() {
    is_scan_started_ = true;
    return true;
  }
  virtual bool stopScan() {
    is_scan_started_ = false;
    return true;
  }
  virtual bool dispose() { return true; }

  virtual bool checkHealth() { return true; }
  virtual void getValidAngleRange(float * min_angle_rad, float * max_angle_rad) {
    *min_angle_rad = min_angle_rad_;
    *max_angle_rad = max_angle_rad_;
  }
  virtual int getValidDataCount() { return valid_data_count_; }
  // Should call updateScan() before calling this function.
  virtual void getScanMessage(Navigation::ScanMessage* scan_msg) { *scan_msg = scan_msg_; }
  virtual void updateScan() = 0;

 protected:
  bool is_initialzed_;
  bool is_scan_started_;
  Navigation::ScanMessage scan_msg_;
  // TODO(meng): need to be used
  int valid_data_count_;
  float min_angle_rad_;
  float max_angle_rad_;
};

#endif  // XP_INCLUDE_XP_WORKER_SCANNER_H_
