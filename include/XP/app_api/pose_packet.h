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

#ifndef XP_INCLUDE_XP_APP_API_POSE_PACKET_H_
#define XP_INCLUDE_XP_APP_API_POSE_PACKET_H_

#include <Eigen/Core>
#include <cstdlib>
#include <functional>
#include <vector>
#include <chrono>
#include <cmath>

namespace XP_TRACKER {

/**
 * 2D Position
 */
struct V2 {
  V2() :
    x(0), y(0) {}
  V2(const float _x, const float _y) :
    x(_x), y(_y) {}
  float x;
  float y;
};

/**
 * 3D Position
 */
struct V3 {
  V3() :
    x(0), y(0), z(0) {}
  V3(const float _x, const float _y, const float _z) :
    x(_x), y(_y), z(_z) {}
  float x;
  float y;
  float z;
};

/**
 * Rotation (quaternion)
 */
struct V4 {
  V4() :
    x(0), y(0), z(0), w(1) {}
  V4(const float _x, const float _y, const float _z, const float _w) :
    x(_x), y(_y), z(_z), w(_w) {}
  float x;
  float y;
  float z;
  float w;
};
/**
 * Rotation (Euler)
 */
struct EulerAngleRPY {
  EulerAngleRPY() :
    ea_yaw(0), ea_pitch(0), ea_roll(0) {}
  EulerAngleRPY(const float _yaw, const float _pitch, const float _roll) :
    ea_yaw(_yaw), ea_pitch(_pitch), ea_roll(_roll) {}
  float ea_yaw;
  float ea_pitch;
  float ea_roll;
};

#define V1_LENGTH (sizeof(ServerPktV1) - 2 * sizeof(int32_t))
struct ServerPktV1 {
  ServerPktV1() :
    version(1),
    length(V1_LENGTH),
    vendorSendTs(0),
    vendorRecvTs(0),
    sensorTs(0),
    trackingStatus(1) {}
  int32_t version;
  int32_t length;
  int64_t vendorSendTs;
  int64_t vendorRecvTs;
  int64_t sensorTs;
  V3 pos;
  V4 rot;
  int32_t trackingStatus;
};

/**
  * \brief The state of vio
  * position [meter]
  * orientation (euler angle:RPY)[rad]
  * linear_velocity [meter/s], represent in current camera coordinate
  * angular_velocity [rad/s], represent in current camera coordinate
  * feature_number (matched feature for current frame) 
 */
// [note] the pose is the transformation between World and Device
// the world coordinate is: z-up, x-right, y-front
struct VioState {
  VioState() :
    matched_feature_number(0) {
  }
  VioState& operator=(const VioState& state_) {
    position.x = state_.position.x;
    position.y = state_.position.y;
    position.z = state_.position.z;
    orientation.ea_yaw = state_.orientation.ea_yaw;
    orientation.ea_pitch = state_.orientation.ea_pitch;
    orientation.ea_roll = state_.orientation.ea_roll;
    linear_velocity.x = state_.linear_velocity.x;
    linear_velocity.y = state_.linear_velocity.y;
    linear_velocity.z = state_.linear_velocity.z;
    angular_velocity.x = state_.angular_velocity.x;
    angular_velocity.y = state_.angular_velocity.y;
    angular_velocity.z = state_.angular_velocity.z;
    matched_feature_number = state_.matched_feature_number;
    timestamp_sec = state_.timestamp_sec;
    timestamp_nsec = state_.timestamp_nsec;
  }
  V3 position;
  EulerAngleRPY orientation;
  V3 linear_velocity;
  V3 angular_velocity;
  int matched_feature_number;
  // timestamp
  uint32_t timestamp_sec;
  uint32_t timestamp_nsec;
};

// TODO(mingyu): Add UDP signatures for GuideMessage and ServerPktV1
/**
 * distance [meter]
 * degree [-180 to 180 degree]
 * With Z:up
 * degree > 0: Turn left
 * degree < 0: Turn right
 */
struct GuideMessage {
  enum Status {
    FAIL = 0,
    OK = 1,
    STOP = 2,
    FINISH = 3,
    LOST = 4,
    OBSTACLE_AVOID = 5,
    MANUAL = 6,
    FORCE_RELOC = 7
  };
  GuideMessage() :
    distance(0), degree(0), vel(0), angular_vel(0), target_id(-1), status(FAIL) {
  }
  explicit GuideMessage(const Status stat) :
    distance(0), degree(0), vel(0), angular_vel(0), target_id(-1), status(stat) {
  }
  GuideMessage(const float dist, const float deg) :
    distance(dist), degree(deg), vel(0), angular_vel(0), target_id(-1), status(OK) {
  }
  float distance;
  float degree;
  float vel;
  float angular_vel;
  int target_id;  // Not necessary. Currently, only used in RosXiaoduActuator.
  Status status;  // enum should be stored as int32_t
};

// TODO(hangmeng): unify the data structure with Xiaodu Group
// TODO(hangmeng): fix the comments
// TODO(hangmeng): need coordinate transformation
/**
 * x: forward [meter]
 * y: left [meter]
 * yaw [-M_PI to M_PI]
 */
struct WheelOdomMessage {
  enum Status {
    FAIL = 0,
    OK = 1,
    CLEAR = 2  // set wheel odometry to zero
  };
  WheelOdomMessage() :
    x(0), y(0), yaw(0), linear_vel(0), angular_vel(0), status(FAIL) {
  }
  explicit WheelOdomMessage(const Status stat) :
    x(0), y(0), yaw(0), linear_vel(0), angular_vel(0), status(stat) {
  }
  WheelOdomMessage(const float input_x, const float input_y, const float input_yaw,
                   const float input_linear_vel, const float input_angular_vel) :
    x(input_x), y(input_y), yaw(input_yaw),
    linear_vel(input_linear_vel), angular_vel(input_angular_vel),
    status(FAIL) {
  }
  float x;
  float y;
  float yaw;
  float linear_vel;
  float angular_vel;
  // TODO(hangmeng): currently not used
  Status status;
};

/**
 * position.y: forward [meter]
 * position.x: right [meter]
 * yaw [-M_PI to M_PI], default: 0.5 M_PI, because the heading is towards +y
 */
// TODO(Mingyu): Move this *state* to data_atom/basic_datatype.h ??
// TODO(Mingyu): Unify the odometry convention and probably do the transform
//               when fusing odometry pose w/ SLAM pose.
struct WheelOdomState {
  WheelOdomState() :
    linear_velocity(0),
    angular_velocity(0),
    ts(std::chrono::steady_clock::now()) {
    A0_T_Ai.setIdentity();
  }
  void reset(const std::chrono::steady_clock::time_point& cur_ts) {
    A0_T_Ai.setIdentity();
    linear_velocity = 0;
    angular_velocity = 0;
    ts = cur_ts;
  }
  void reset() {
    this->reset(std::chrono::steady_clock::now());
  }
  void updateA0_T_AiFrom2dPose(float x, float y, float yaw) {
    // +x: front, +y: left
    A0_T_Ai << cosf(yaw), -sinf(yaw), x,
               sinf(yaw),  cosf(yaw), y,
                       0,          0, 1;
  }
  inline float getX() {
    return A0_T_Ai(0, 2);
  }
  inline float getY() {
    return A0_T_Ai(1, 2);
  }
  inline float getYaw() {
    return atan2f(A0_T_Ai(1, 0), A0_T_Ai(0, 0));
  }

  Eigen::Matrix3f A0_T_Ai;
  float linear_velocity;  // m/s
  float angular_velocity;  // rad/s
  // TODO(hangmeng): unify the time with other sensor data like VIO
  std::chrono::steady_clock::time_point ts;
};

/**
 * \brief Callback function for getting walk guide data
 * \param guide_message The guide message produced by SDK
 */
typedef
std::function<void(const XP_TRACKER::GuideMessage& guide_message)> GuideMessageCallback;
typedef
std::function<bool(XP_TRACKER::WheelOdomMessage * wheel_odom_message)> WheelOdomMessageCallback;

// UDP message encoding obstacle information.
// The image is divided into a 2x3 grid and nonzero means obstacle.
struct ObstacleMessage {
  int obstacle_block[6];
};

/**
 * \brief Command sent by a host and recieved by tracking
 * since udp only receives bytes,
 * it's very likely to make a mistake where some date received by UDP is erroneously converted into
 * a packet that the data is actually not.
 * By checking the signature,
 * we can detect the conversion error if the converted data does share this signature value.
 */
#define COMMAND_MESSAGE_SIGNATURE 0x200
struct CommandMessage {
  enum Command {
    UNKNOWN = 0,
    GOTO = 1,  // tell path follower where to go. Use val to set tag
    STOPPED = 2,  // tell SLAM the device has stopped moving
    MOVING = 3,  // tell SLAM the device is moving
    EXIT = 4,  // stop the program
  };
  CommandMessage() :
    val(0),
    command(UNKNOWN) {
  }
  // in case a receiver needs to distinguish what packat is received
  // this is just a random num shared by all CommandMessage
  const int signature = COMMAND_MESSAGE_SIGNATURE;
  int val;
  Command command;  // enum should be stored as int32_t
};

enum class MotionMode {
  LOOP = 0,
  CONTROL = 1,
  MANUAL = 2,
  NONE = 3,
};

enum class LinearVelocityLevel {
  BACKWARD_FAST = -3,
  BACKWARD_NORMAL = -2,
  BACKWARD_SLOW = -1,
  IDLE = 0,
  FORWARD_SLOW = 1,
  FORWARD_NORMAL = 2,
  FORWARD_FAST = 3,
};
enum class AngularVelocityLevel {
  RIGHT_FAST = -3,
  RIGHT_NORMAL = -2,
  RIGHT_SLOW = -1,
  IDLE = 0,
  LEFT_SLOW = 1,
  LEFT_NORMAL = 2,
  LEFT_FAST = 3,
};
}  // namespace XP_TRACKER
#endif  // XP_INCLUDE_XP_APP_API_POSE_PACKET_H_
