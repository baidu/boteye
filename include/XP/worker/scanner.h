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

#ifndef XP_INCLUDE_XP_WORKER_SCANNER_H_
#define XP_INCLUDE_XP_WORKER_SCANNER_H_

#include <navigation/navigation_type.h>
#include <XP/helper/param_internal.h>  // for NaviParam
#include <string>

class Scanner {
 public:
  explicit Scanner(const XP::NaviParam::LidarConfig_t &lidar_param) :
      is_initialzed_(false),
      is_scan_started_(false),
      min_valid_scan_dist_for_robot_(lidar_param.min_valid_scan_dist_for_robot),
      scan_size_(lidar_param.scan_size),
      scan_time_(lidar_param.scan_time),
      time_increment_(lidar_param.time_increment),
      range_min_(lidar_param.range_min),
      range_max_(lidar_param.range_max),
      angle_min_(lidar_param.angle_min),
      angle_max_(lidar_param.angle_max),
      angle_increment_(lidar_param.angle_increment) {
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
  // Should call updateScan() before calling this function.
  virtual void getScanMessage(Navigation::ScanMessage *scan_msg) const { *scan_msg = scan_msg_; }
  virtual void updateScan() = 0;
  virtual bool isBeamValid(const float distance_beam) const {
    return distance_beam > min_valid_scan_dist_for_robot_;
  }
  virtual int getScanSize() const { return scan_size_; }
  virtual float getScanDuration() const { return scan_time_; }
  virtual float getTimeIncrement() const { return time_increment_; }
  virtual float getRangeMin() const { return range_min_; }
  virtual float getRangeMax() const { return range_max_; }
  virtual float getAngleMin() const { return angle_min_; }
  virtual float getAngleMax() const { return angle_max_; }
  virtual float getAngleIncrement() const { return angle_increment_; }

 protected:
  bool is_initialzed_;
  bool is_scan_started_;
  Navigation::ScanMessage scan_msg_;
  const float min_valid_scan_dist_for_robot_;
  int scan_size_;
  const float scan_time_;
  const float time_increment_;
  const float range_min_;
  const float range_max_;
  float angle_min_;
  float angle_max_;
  const float angle_increment_;
};

#endif  // XP_INCLUDE_XP_WORKER_SCANNER_H_
