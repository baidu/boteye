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

#ifndef XP_INCLUDE_XP_WORKER_ACTUATOR_H_
#define XP_INCLUDE_XP_WORKER_ACTUATOR_H_

#include <XP/app_api/pose_packet.h>  // for GuideMessageCallback and GuideMessage
#include <thread>
#include <mutex>

namespace XP {

// This is an abstract class of an actuator that converts the physics motion control
// into robot-platform specific control signals
class Actuator {
 public:
  Actuator() {}
  virtual ~Actuator() {}

  virtual bool init();
  virtual bool run() = 0;
  virtual bool update() = 0;  // Update the sensors on this actuator
  virtual bool control(const XP_TRACKER::GuideMessage& guide_message) = 0;
  virtual bool stopActProcess();
  virtual bool allowMoving(const bool allow_moving);
  virtual bool isInit() const { return is_init_; }

  // The getters actually get the *cached* sensor data of the actuator.
  // The base class implementation returns false for all sensors.
  virtual bool getUltrasoundObstacle() { return false; }
  // The wheel odometry reading: +positive means forward for both left & right readings
  virtual bool getWheelEncoders_lr(int* l_reading, int* r_reading) { return false; }
  // TODO(hangmeng): re-difine the wheel odom's data structure
  // which will be different from WheelOdomMessage including velocity data
  virtual bool getWheelOdom(XP_TRACKER::WheelOdomState* wheel_odom_state) { return false; }

 protected:
  bool is_init_ = false;
  bool is_running_ = false;

  // Mutex to protect the actuator sensor data I/O
  std::mutex actuator_sensor_lock_;
  XP_TRACKER::WheelOdomState wheel_odom_state_;
  bool ultrasound_obstacle_ = false;
  int l_reading_ = 0;
  int r_reading_ = 0;
};

// This actuator sends out control signals via UDP or serial port
class SampleActuator : public Actuator {
 public:
  SampleActuator(const XP_TRACKER::GuideMessageCallback& g_callback,
                 const XP_TRACKER::WheelOdomMessageCallback& wo_callback);
  bool run();
  bool update();
  bool control(const XP_TRACKER::GuideMessage& guide_message) override;
  bool stopActProcess() override;
  bool getWheelOdom(XP_TRACKER::WheelOdomState* wheel_odom_state) override;

 protected:
  // Need a separate thread for UDP receiving (blocking).
  virtual void threadRecvUDPMessage();
  virtual bool recvUDPWheelOdom(XP_TRACKER::WheelOdomMessage * wheel_odom_message);

  std::thread recv_thread_;
  XP_TRACKER::GuideMessageCallback g_callback_;
  XP_TRACKER::WheelOdomMessageCallback wo_callback_;
};

}  // namespace XP

#endif  // XP_INCLUDE_XP_WORKER_ACTUATOR_H_
