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

#ifndef XP_INCLUDE_XP_APP_API_POSE_PACKET_H_
#define XP_INCLUDE_XP_APP_API_POSE_PACKET_H_

#include <cstdlib>
#include <functional>
namespace XP_TRACKER {

/**
 * Position
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


// distance [meter]
// degree [-180 to 180 degree]
// With Z:up
// degree > 0: Turn left
// degree < 0: Turn right
struct GuideMessage {
  enum Status {
    FAIL = 0,
    OK = 1,
    STOP = 2,
    FINISH = 3,
  };
  GuideMessage() :
    distance(0), degree(0), vel(0), angular_vel(0), status(FAIL) {
  }
  explicit GuideMessage(const Status stat) :
    distance(0), degree(0), vel(0), angular_vel(0), status(stat) {
  }
  GuideMessage(const float dist, const float deg) :
    distance(dist), degree(deg), status(OK) {
  }
  float distance;
  float degree;
  float vel;
  float angular_vel;
  Status status;  // enum should be stored as int32_t
};
/**
 * \brief Callback function for getting walk guide data
 * \param guide_message The guide message produced by SDK
 */
typedef std::function<void(const XP_TRACKER::GuideMessage& guide_message)> GuideMessageCallback;

// UDP message encoding obstacle information.
// The image is divided into a 2x3 grid and nonzero means obstacle.
struct ObstacleMessage {
  int obstacle_block[6];
};
// command sent by a host and recieved by tracking
/*
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
}  // namespace XP_TRACKER
#endif  // XP_INCLUDE_XP_APP_API_POSE_PACKET_H_
