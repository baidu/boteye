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

#include <XP/app_api/pose_packet.h>
#include <plotting_utils.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>  // for memset
#include <unistd.h>  // for usleep
#include <string>
#include <thread>
#include <mutex>
#define MAXNAME 1024

DEFINE_bool(no_display, false, "Do not use viz3d display");
DEFINE_int32(client_port, -1, "Set to e.g. 8888 to enable HelloVR");
DEFINE_int32(server_port, 8887, "app tracking output port");
// copied from Eigen implementation Quaternion.h
// template<class Derived>
// inline typename QuaternionBase<Derived>::Matrix3
// QuaternionBase<Derived>::toRotationMatrix(void) const

cv::Matx33f computeRotationMatrix(float x, float y, float z, float w) {
  cv::Matx33f res;
  const float tx  = 2.f * x;
  const float ty  = 2.f * y;
  const float tz  = 2.f * z;
  const float twx = tx * w;
  const float twy = ty * w;
  const float twz = tz * w;
  const float txx = tx * x;
  const float txy = ty * x;
  const float txz = tz * x;
  const float tyy = ty * y;
  const float tyz = tz * y;
  const float tzz = tz * z;

  res(0, 0) = 1.f - (tyy + tzz);
  res(0, 1) = txy - twz;
  res(0, 2) = txz + twy;
  res(1, 0) = txy + twz;
  res(1, 1) = 1.f - (txx + tzz);
  res(1, 2) = tyz - twx;
  res(2, 0) = txz - twy;
  res(2, 1) = tyz + twx;
  res(2, 2) = 1.f - (txx + tyy);

  return res;
}

bool running_flag = true;
std::mutex g_cam_pose_mutex;
cv::Affine3f g_cam_pose;
void viz3d_thread() {
#ifndef HAS_OPENCV_VIZ
  LOG(ERROR) << "Opencv Viz cannot be used";
  return;
#else
  cv::Matx33f K;
  K << 400, 0, 320, 0, 400, 240, 0, 0, 1;
  PoseDrawer3D pose_viewer_3d(2, K);
  while (running_flag) {
    cv::Affine3f cam_pose;
    {
      // copy pose and release lock
      std::lock_guard<std::mutex> lock(g_cam_pose_mutex);
      cam_pose = g_cam_pose;
    }
    pose_viewer_3d.viz3d_once(cam_pose);
    if (pose_viewer_3d.key_pressed() == 27) {
      // ESC
      running_flag = false;
    }
  }
#endif
}
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  FLAGS_colorlogtostderr = 1;
  int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    perror("socket error");
    exit(-1);
  }
  int socket_fd2 = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd2 < 0) {
    perror("socket2 error");
    exit(-1);
  }

  struct sockaddr_in addr;
  socklen_t addr_length = sizeof(addr);
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(FLAGS_server_port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive data from any IP addr
  // Binding (to tracking app)
  if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Fail to connect to tracking app");
    exit(-1);
  }

  struct sockaddr_in client_addr;
  if (FLAGS_client_port > 0) {
    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(FLAGS_client_port);
    client_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive data from any IP addr
    // Binding (to pose client)
    if (bind(socket_fd2, (struct sockaddr*)&client_addr, sizeof(client_addr)) < 0) {
      perror("Fail to connect to pose client");
      exit(-1);
    }
    // Wait for 'Hello' from HelloVR
    printf("Wait for pose client to connect\n");
    char buf[BUFSIZ];
    int nbytes = recvfrom(socket_fd2, &buf, MAXNAME, 0,
                          (struct sockaddr*)&client_addr, &addr_length);
    if (nbytes < 0) {
      perror("could not read datagram!!");
      exit(-1);
    }
    printf("Received data form client %s : %d\n",
           inet_ntoa(client_addr.sin_addr), htons(client_addr.sin_port));
    printf("%s\n", buf);
  }
  // start viz thread
  std::unique_ptr<std::thread> viz_3d_thread_ptr;
  if (!FLAGS_no_display) {
    viz_3d_thread_ptr.reset(new std::thread(&viz3d_thread));
  }
  // Start receiving (and forwarding)
  printf("Start receiving...\n");
  int64_t last_sending_time = 0;
  while (running_flag) {
    XP_TRACKER::ServerPktV1 pkt;
    int len = recvfrom(socket_fd, &pkt, sizeof(XP_TRACKER::ServerPktV1), 0,
                       (struct sockaddr*)&addr, &addr_length);

    if (len != sizeof(XP_TRACKER::ServerPktV1)) {
      printf("Error in recvfrom from %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    } else {
      // printf("recvfrom from %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
      // printf("pos = [%f %f %f]\n", pkt.version, pkt.pos.x, pkt.pos.y, pkt.pos.z);

      // Forward to pose client
      if (FLAGS_client_port > 0) {
        if (sendto(socket_fd2, &pkt, sizeof(pkt), 0,
                   (struct sockaddr*)&client_addr, addr_length) < 0) {
          perror("Could not send datagram to pose client!!\n");
          continue;
        }
      }
      if (!FLAGS_no_display) {
        if (pkt.sensorTs > last_sending_time) {
          cv::Affine3f cam_pose;
          cam_pose.translation(cv::Vec3f(pkt.pos.x, pkt.pos.y, pkt.pos.z));
          cv::Matx33f R = computeRotationMatrix(pkt.rot.x,
                                                pkt.rot.y,
                                                pkt.rot.z,
                                                pkt.rot.w);
          cam_pose.rotation(R);
          // copy pose and release lock
          std::lock_guard<std::mutex> lock(g_cam_pose_mutex);
          g_cam_pose = cam_pose;
          last_sending_time = pkt.sensorTs;
        } else {
          LOG(ERROR) << "pkt.sensorTs " << pkt.sensorTs << " < "
                     << " last_sending_time " << last_sending_time;
        }
      }
    }
    // usleep(1000);  // Seems redundant
  }
  if (viz_3d_thread_ptr != nullptr) {
    viz_3d_thread_ptr->join();
  }
  return 0;
}
