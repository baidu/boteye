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

#ifndef XP_INCLUDE_XP_WORKER_SCANNER_IMPL_H_
#define XP_INCLUDE_XP_WORKER_SCANNER_IMPL_H_

#ifdef HAS_ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#endif

#ifdef HAS_RPLIDAR
#include <rplidar.h>
#endif  // HAS_RPLIDAR

#ifdef HAS_LSLIDAR
#include <ls01b_driver.h>
#include <ls01d_driver.h>
#endif  // HAS_LSLIDAR

#ifdef HAS_YDLIDAR
#include <CYdLidar.h>
#endif  // HAS_YDLIDAR

#include <XP/worker/scanner.h>
#include <string>
#include <chrono>
#include <vector>

// single thread
class NullScanner : public Scanner {
 public:
  explicit NullScanner(const XP::NaviParam::LidarConfig_t &lidar_param)
    : Scanner(lidar_param) {}
  bool init() override;
  void updateScan() override;
};

#ifdef HAS_RPLIDAR
class RPLidarScanner : public Scanner {
 public:
  explicit RPLidarScanner(const XP::NaviParam::LidarConfig_t &lidar_param);
  ~RPLidarScanner() override;

  bool init() override;
  bool startScan() override;
  bool stopScan() override;
  bool dispose() override;
  bool checkHealth() override;
  void updateScan() override;

 protected:
  void createDriver();
  bool connectDriver();

  const char *serial_dev_;
  uint32_t baudrateArray_[2];
  uint32_t baudrate_;
  rplidar_response_device_info_t devinfo_;

  rp::standalone::rplidar::RPlidarDriver *device_;
};
#endif  // HAS_RPLIDAR

#ifdef HAS_LSLIDAR
class LSLidarScanner : public Scanner {
 public:
  explicit LSLidarScanner(const XP::NaviParam::LidarConfig_t &lidar_param);
  ~LSLidarScanner() override;

  bool init() override;
  bool startScan() override;
  bool stopScan() override;
  bool dispose() override;
  void updateScan() override;

 protected:
  std::string serial_dev_;

  io_driver *device_ls01d_;
  lidar_driver *device_ls01b_;
};
#endif  // HAS_LSLIDAR

#ifdef HAS_YDLIDAR
class YDLidarScanner : public Scanner {
 public:
  explicit YDLidarScanner(const XP::NaviParam::LidarConfig_t &lidar_param);
  ~YDLidarScanner() override;

  bool init() override;
  bool startScan() override;
  bool stopScan() override;
  void updateScan() override;

 protected:
  CYdLidar device_;
  std::string port_;
  int baudrate_;
  bool resolution_fixed_;
  bool intensities_;
  bool low_exposure_;
  bool auto_reconnect_;
  bool reversion_;
  int samp_rate_;
  int frequency_;
  std::vector<float> ignore_array_;

  bool scan_first_;
  uint64_t ydlidar_sys_time_start_nanosec_;
  std::chrono::time_point<std::chrono::steady_clock> scan_sample_start_tp_;
};
#endif  // HAS_YDLIDAR

#ifdef HAS_ROS
// single thread
class RosScanner : public Scanner {
 public:
  explicit RosScanner(const XP::NaviParam::LidarConfig_t &lidar_param);
  ~RosScanner() override;
  bool init() override;
  bool startScan() override;
  bool stopScan() override;
  bool dispose() override;
  bool checkHealth() override;
  void updateScan() override;

 protected:
  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan);
  void diagCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &diag);

  bool scan_first_;
  int64_t ros_header_time_nanosec_;
  std::chrono::time_point<std::chrono::steady_clock> scan_sample_start_tp_;
  bool is_lidar_healthy_;
  ros::Subscriber diag_sub_;
  ros::Subscriber scan_sub_;
  const std::string diag_topic_;
  const std::string scan_topic_;
};
#endif

#endif  // XP_INCLUDE_XP_WORKER_SCANNER_IMPL_H_
