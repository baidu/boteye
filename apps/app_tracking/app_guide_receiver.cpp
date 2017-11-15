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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>  // for memset
#include <string>

#define RECEIVING_PORT 8889

int main() {
  int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    perror("socket error");
    exit(-1);
  }

  struct sockaddr_in addr;
  socklen_t addr_length = sizeof(addr);
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(RECEIVING_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive data sent from any IP addr

  // Binding
  if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("Fail to bind socket_fd");
    exit(-1);
  }

  // Start receiving
  printf("Start receiving guide messages at port %d...\n", RECEIVING_PORT);
  while (1) {
    XP_TRACKER::GuideMessage guide_message;
    int len = recvfrom(socket_fd, &guide_message, sizeof(XP_TRACKER::GuideMessage), 0,
                       (struct sockaddr*)&addr, &addr_length);
    if (len != sizeof(XP_TRACKER::GuideMessage)) {
      printf("Error in recvfrom from %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    } else {
      printf("guide msg from %s:%d   [%4.2f, %4.2f, %d]\n",
             inet_ntoa(addr.sin_addr), RECEIVING_PORT,
             guide_message.vel, guide_message.angular_vel, guide_message.status);
    }
  }

  return 0;
}
