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

#ifndef XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_
#define XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_

#include <XP/helper/param.h>  // for NaviParam::ActuatorConfig_t
#include <XP/worker/actuator.h>
#include <XP/worker/scanner_impl.h>
#include <Eigen/Core>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>

namespace XP {

namespace internal {
// Convert input buffer into a string with range [start, end)
std::string buf_to_string(uint8_t *buf, int start, int end);

}  // namespace internal

class EaiActuator : public Actuator {
 public:
  EaiActuator(const NaviParam::ActuatorConfig_t &actuator_param,
              const NaviParam::LidarConfig_t &lidar_param);
  ~EaiActuator();

  bool init() override;
  void update() override;
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage &guide_message) override;
  bool updateWheelOdom();
  bool updateWheelEncoders_lr(int *l_reading, int *r_reading);

 private:
  // Device related
  std::string dev_str_;
  int fd_;
  uint8_t ultrasound_flag_;  // Each bit indicates an ultrasound_sensor. e.g. 0x1 is A 0x3 is AB
  float axle_length_;  // Distance between two wheels
  float wheel_diameter_;
  int encoder_ticks_num_;  // the amount of ticks on a whole encoder circle
  bool allow_forced_moving_;  // Whether or not the Dilili desk can be forced moving around.
  bool first_time_force_not_moving_;
  std::mutex serial_port_mutex_;  // Protect the serial port accessing in multithread

  bool reverse_heading_;  // If true, the heading of KincoActuator is reversed, i.e., XP config

  std::chrono::steady_clock::time_point latest_odom_tp_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};

// This actuator controls a Kinco base
class KincoActuator : public Actuator {
 public:
  KincoActuator(const NaviParam::ActuatorConfig_t &actuator_param,
                const NaviParam::LidarConfig_t &lidar_param);
  ~KincoActuator();

  // Due to the serial port nature, it's recommended to use all
  // sendKincoCommand in one thread, i.e., update() and control() in one thread
  bool init() override;
  void update() override;
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage &guide_message) override;

 public:
  // Dilili desk specific functions
  bool deskUp();
  bool deskDown();
  bool deskStop();
  bool deskResetAlarm();
  bool lightSwitch(int gry, bool on_off);

  bool resetAndEnableWheels();
  bool powerOnSeparateWheels();   // before moving
  bool powerOffSeparateWheels();  // so you can push the desk
  bool resetAndEnableSeparateWheelControl();
  bool allowForcedMoving(bool allow_forced_moving);
  bool sendKincoCommand(uint32_t cmd, int32_t data,
                        const std::string &ns = "",
                        int id = 0x01,  // wheel 2 is 0x02
                        uint32_t *res = nullptr);
  bool setWheelRps_lr(int rps_l, int rps_r);

 private:
  bool updateUltrasoundObstacle();
  bool updateWheelOdom();
  bool updateWheelEncoders_lr(int *l_reading, int *r_reading);

 private:
  // Device related
  std::string dev_str_;
  std::string actuator_str_;
  int fd_;
  uint8_t ultrasound_flag_;  // Each bit indicates an ultrasound_sensor. e.g. 0x1 is A 0x3 is AB
  float axle_length_;  // Distance between two wheels
  float wheel_diameter_;
  int encoder_ticks_num_;  // the amount of ticks on a whole encoder circle
  bool allow_forced_moving_;  // Whether or not the Dilili desk can be forced moving around.
  std::mutex serial_port_mutex_;  // Protect the serial port accessing in multithread

  bool reverse_heading_;  // If true, the heading of KincoActuator is reversed, i.e., XP config

  std::chrono::steady_clock::time_point latest_odom_tp_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};

class GyroorActuator : public Actuator {
 private:
  enum class Payload {
    CONTROL = 1,
    LOCK = 2,
    UNLOCK = 3,
    STATUS = 4,
  };

 public:
  GyroorActuator(const NaviParam::ActuatorConfig_t &actuator_param,
                 const NaviParam::LidarConfig_t &lidar_param);
  ~GyroorActuator();

  bool init() override;
  void update();
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage &guide_message) override;

 public:
  bool powerOnSeparateWheels();   // before moving
  bool powerOffSeparateWheels();  // so you can push the desk
  bool allowForcedMoving(bool allow_forced_moving);
  bool sendSerialCommand(const Payload &payload,
                         const std::vector<uint8_t> &data,
                         const std::string &ns);

 private:
  void updateUltrasoundObstacle();
  void updateWheelOdom();

 private:
  // Device related
  std::string dev_str_;
  int fd_;
  uint8_t ultrasound_channels_[6];
  uint8_t ultrasound_flag_;  // Each bit indicates an ultrasound_sensor. e.g. 0x1 is A 0x3 is AB
  float axle_length_;  // Distance between two wheels
  float wheel_diameter_;
  int encoder_ticks_num_;  // the amount of ticks on a whole encoder circle
  bool allow_forced_moving_;  // Whether or not the Gyroor robot can be forced moving around.
  std::mutex serial_port_mutex_;  // Protect the serial port accessing in multithread

  bool reverse_heading_;  // If true, the heading of GyroorActuator is reversed, i.e., XP config

  std::chrono::steady_clock::time_point latest_odom_tp_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};

#ifdef HAS_ROS
// This actuator controls a ROS base
class RosActuator : public Actuator {
 public:
  RosActuator(const NaviParam::ActuatorConfig_t &actuator_param,
              const NaviParam::LidarConfig_t &lidar_param);
  ~RosActuator();

  bool init() override;
  bool run() override;
  void update() override;
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage &guide_message) override;

 protected:
  virtual void odomCallback(const nav_msgs::OdometryConstPtr &odom);
  virtual void diagCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &diag);
  virtual void processWheelOdom(const float wo_x, const float wo_y, const float wo_yaw,
                                const float wo_linear_vel, const float wo_angular_vel,
                                const int64_t wo_tp_nanosec);

 protected:
  // Device related
  bool odom_first_;
  int64_t ros_header_time_nanosec_;
  Eigen::Matrix3f WO_origin_T_P0_;
  std::chrono::time_point<std::chrono::steady_clock> odom_sample_start_tp_;
  int battery_diag_;
  const std::string odom_topic_;
  const std::string diag_topic_;
  const std::string cmd_topic_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber diag_sub_;
};

class RosXiaoduActuator : public RosActuator {
 public:
  RosXiaoduActuator(const NaviParam::ActuatorConfig_t &actuator_param,
                    const NaviParam::LidarConfig_t &lidar_param);
  ~RosXiaoduActuator();
  bool init() override;
  bool control(const XP_TRACKER::GuideMessage &guide_message) override;

 protected:
  void odomCallback(const nav_msgs::OdometryConstPtr &odom) override;
  void diagCallback(const diagnostic_msgs::DiagnosticArrayConstPtr &diag) override;

 private:
  const std::string navi_status_topic_;
  ros::Publisher navi_status_pub_;
};

#endif  // HAS_ROS
}  // namespace XP
#endif  //  XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_
