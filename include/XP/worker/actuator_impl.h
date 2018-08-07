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

#ifndef XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_
#define XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_

#include <XP/helper/param.h>  // for NaviParam::ActuatorConfig_t
#include <XP/worker/actuator.h>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>

namespace XP {

namespace internal {
// Convert input buffer into a string with range [start, end)
std::string buf_to_string(uint8_t* buf, int start, int end);

}  // namespace internal

class EaiActuator : public Actuator {
 public:
  explicit EaiActuator(const NaviParam::ActuatorConfig_t& actuator_param);
  ~EaiActuator();

  bool init() override;
  bool run();
  bool update();
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage& guide_message) override;
  bool getUltrasoundObstacle() override;
  bool getWheelEncoders_lr(int* l_reading, int* r_reading) override;
  bool getWheelOdom(XP_TRACKER::WheelOdomState* wheel_odom_state) override;
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

  std::chrono::steady_clock::time_point latest_odom_ts_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};

// This actuator controls a Kinco base
class KincoActuator : public Actuator {
 public:
  explicit KincoActuator(const NaviParam::ActuatorConfig_t& actuator_param);
  ~KincoActuator();

  // Due to the serial port nature, it's recommended to use all
  // sendKincoCommand in one thread, i.e., update() and control() in one thread
  bool init() override;
  bool run();
  bool update();
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage& guide_message) override;
  bool getUltrasoundObstacle() override;
  bool getWheelEncoders_lr(int* l_reading, int* r_reading) override;
  bool getWheelOdom(XP_TRACKER::WheelOdomState* wheel_odom_state) override;

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
                        const std::string& ns = "",
                        int id = 0x01,  // wheel 2 is 0x02
                        uint32_t* res = nullptr);
  bool setWheelRps_lr(int rps_l, int rps_r);

 private:
  bool updateUltrasoundObstacle();
  bool updateWheelOdom();
  bool updateWheelEncoders_lr(int* l_reading, int* r_reading);

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

  std::chrono::steady_clock::time_point latest_odom_ts_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};

class XiaoduActuator : public SampleActuator {
 public:
  XiaoduActuator(const XP_TRACKER::GuideMessageCallback& g_callback,
                 const XP_TRACKER::WheelOdomMessageCallback& wo_callback);

  bool run() override;

 protected:
  void threadRecvUDPMessage() override;
  bool recvUDPWheelOdom(XP_TRACKER::WheelOdomMessage * wheel_odom_message) override;
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
  explicit GyroorActuator(const NaviParam::ActuatorConfig_t& actuator_param);
  ~GyroorActuator();

  bool init() override;
  bool run();
  bool update();
  bool stopActProcess() override;
  bool control(const XP_TRACKER::GuideMessage& guide_message) override;
  bool getUltrasoundObstacle() override;
  bool getWheelEncoders_lr(int* l_reading, int* r_reading) override;
  bool getWheelOdom(XP_TRACKER::WheelOdomState* wheel_odom_state) override;

 public:
  bool powerOnSeparateWheels();   // before moving
  bool powerOffSeparateWheels();  // so you can push the desk
  bool allowForcedMoving(bool allow_forced_moving);
  bool sendSerialCommand(const Payload &payload,
                         const std::vector<uint8_t> &data,
                         const std::string& ns);

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

  std::chrono::steady_clock::time_point latest_odom_ts_;
  int prev_tick_counts_l_;
  int prev_tick_counts_r_;
  float dist_per_tick_;
};
}  // namespace XP
#endif  //  XP_INCLUDE_XP_WORKER_ACTUATOR_IMPL_H_
