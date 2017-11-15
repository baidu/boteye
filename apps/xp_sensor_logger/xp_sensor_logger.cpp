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
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <glog/logging.h>
#include <XP/helper/shared_queue.h>
#include <XP/helper/timer.h>
#include <XP/helper/param.h>
#include <XP/helper/tag_detector.h>
#include <XP/driver/LI_sensor.h>
#include <XP/driver/XP_sensor.h>
#include <XP/driver/xp_aec_table.h>
#include <XP/util/depth_utils.h>
#include <XP/util/feature_utils.h>
#include <XP/util/image_utils.h>
#ifdef __linux__
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
using std::cout;
using std::endl;

DEFINE_string(record_path, "", "path to save images. Set empty to disable saving");
DEFINE_string(calib_yaml, "", "load calib file");
DEFINE_string(dev_id, "", "which dev to open. Empty enables auto mode");
DEFINE_string(sensor_type,
#ifdef __linux__  // predefined by gcc
              "XP", "LI or XP or XP2 or XP3 or OCV(openCV) or UDP (receiving)");
#else
              "OCV", "OCV (openCV)");
#endif
DEFINE_int32(udp_port, 8882, "UDP port");
DEFINE_string(udp_ip, "", "Set non-empty value to send to remote");
DEFINE_bool(spacebar_mode, false, "only save img when press space bar");
DEFINE_bool(undistort, false, "whether or not undistort img");
DEFINE_bool(verbose, false, "whether or not log more info");
DEFINE_bool(show_horizontal_line, false, "show green horizontal lines for disparity check");
DEFINE_bool(calib_verify, false, "Use april tag board to verify calib result");
DEFINE_bool(orb_verify, false, "Use ORB feature matching to verify calib result");
DEFINE_bool(headless, false, "Do not show windows");
DEFINE_bool(old_aec_method, false, "Use the org aec method");
DEFINE_bool(show_hist, false, "Show image histogram (left view)");
DEFINE_bool(save_image_bin, false, "Do not save image bin file");
DEFINE_bool(imu_from_image, false, "Do not load imu from image");


#ifdef __ARM_NEON__
DEFINE_int32(cpu_core, 6, "bind program to run on specific core[0 ~ 7],"
             "being out-of-range indicates no binding, only valid on ARM platform");
#endif
struct V4l2BufferData {
  int counter;
  std::shared_ptr<std::vector<uint8_t>> img_data_ptr;
  V4l2BufferData() {
    img_data_ptr.reset(new std::vector<uint8_t>);
  }
};
XP::shared_queue<XP_DRIVER::LI_SENSOR::XP_20608_data> imu_samples_queue;
XP::shared_queue<V4l2BufferData> imgs_queue;

std::atomic<bool> run_flag;

// udp packet
// most UDP protocal has a data size limit. So we downsample the image
struct UdpImg {
  float img_time;
  uint8_t img_l_128x96[128 * 96];
  uint8_t img_r_128x96[128 * 96];
};

// use system time to approx time stamp
std::chrono::time_point<std::chrono::steady_clock> main_start_time;
struct timespec main_start_timespec;
// we use the first imu to approx img time based on img counter
std::atomic<uint64_t> g_first_imu_clock_count;
std::atomic<int> g_imu_rate;
std::atomic<int> g_imu_sample_count;
std::atomic<bool> g_auto_gain;
uint32_t g_gain_val;
uint32_t g_exp_val;
int g_aec_index;  // signed int as the index may go to negative while calculation
std::chrono::time_point<std::chrono::steady_clock> g_imu_sample_start_tp;
std::atomic<uint64_t> g_last_imu_time;
std::atomic<bool> g_pull_imu;
#ifdef __linux__  // predefined by gcc
void thread_receive_udp_imgs() {
  VLOG(1) << "========= thread_receive_udp_imgs loop starts";
  const int img_buffer_size = 20;
  std::vector<std::shared_ptr<std::vector<uint8_t>>>
      img_data_buffers(img_buffer_size);
  for (auto& img_data_buffer : img_data_buffers) {
    img_data_buffer.reset(new std::vector<uint8_t>(640 * 480 * 2, 0));
  }
  int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    perror("socket error");
    exit(-1);
  }
  struct sockaddr_in addr;
  socklen_t addr_length = sizeof(addr);
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(FLAGS_udp_port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive data from any IP addr
  // Binding
  if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Fail to connect to another xp_sensor_logger");
    exit(-1);
  }
  int counter = 0;
  while (run_flag) {
    XP::ScopedLoopProfilingTimer LoopProfilingTimer("thread_receive_udp_imgs", 1);
    VLOG(1) << "========= thread_pull_img loop starts";
    UdpImg udp_img;
    const int len = recvfrom(socket_fd, &udp_img, sizeof(UdpImg), 0,
                             (struct sockaddr*)&addr, &addr_length);
    if (len != sizeof(UdpImg)) {
      printf("Error in recvfrom from %s:%d\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
    } else {
      V4l2BufferData buffer_and_data;
      buffer_and_data.counter = counter;
      buffer_and_data.img_data_ptr = img_data_buffers[counter % img_buffer_size];
      // recover imgs
      for (int i = 0; i < 96; ++i) {
        for (int j = 0; j < 128; ++j) {
          buffer_and_data.img_data_ptr->at(i * 5 * 2 * 640 + j * 5 * 2 + 0) =
              udp_img.img_l_128x96[i * 128 + j];
          buffer_and_data.img_data_ptr->at(i * 5 * 2 * 640 + j * 5 * 2 + 1) =
              udp_img.img_r_128x96[i * 128 + j];
        }
      }
      // copy header
      memcpy(&(buffer_and_data.img_data_ptr->at(0)), udp_img.img_l_128x96, 20);
      // stamp imu data
      XP_DRIVER::XP_SENSOR::stamp_timestamp_in_img(
            &(buffer_and_data.img_data_ptr->at(0)),
            static_cast<uint64_t>(udp_img.img_time * 1000));
      VLOG(1) << "imgs_queue (" << imgs_queue.size() << ") pushing back";
      imgs_queue.push_back(buffer_and_data);
      VLOG(1) << "imgs_queue (" << imgs_queue.size() << ") pushed back";
    }
    ++counter;
  }
}
void thread_pull_img(int fd, struct v4l2_buffer bufferinfo) {
  int counter = 0;
  // set XP cam mode
  if (FLAGS_sensor_type == "XP" || FLAGS_sensor_type == "XP2" || FLAGS_sensor_type == "XP3") {
    // AEC and AGC not working yet. Have to set to 0 and 0
    XP_DRIVER::XP_SENSOR::set_auto_exp_and_gain(fd, 0, 0);
  }
  const int img_buffer_size = 20;
  const size_t raw_img_size = 640 * 480 * 2;
  std::vector<std::shared_ptr<std::vector<uint8_t>>> img_data_buffers(img_buffer_size);
  for (auto& img_data_buffer : img_data_buffers) {
    img_data_buffer.reset(new std::vector<uint8_t>(raw_img_size));
  }
  // start streaming
  while (run_flag) {
    XP::ScopedLoopProfilingTimer LoopProfilingTimer("thread_pull_img", 1);
    VLOG(1) << "========= thread_pull_img loop starts";
    if (FLAGS_sensor_type == "LI"
        || FLAGS_sensor_type == "XP"
        || FLAGS_sensor_type == "XP2"
        || FLAGS_sensor_type == "XP3") {
      uint8_t* data_ptr = nullptr;
      if (!XP_DRIVER::LI_SENSOR::access_next_img_and_queue_next(fd,
                                                                &bufferinfo,
                                                                &data_ptr)) {
        LOG(ERROR) << "access_next_img_and_queue_next failed";
        continue;
      }
      VLOG(1) << "access_next_img_and_queue_next done "
              << " buffer.length = " << bufferinfo.length
              << " data_ptr=" << reinterpret_cast<void*>(data_ptr);
      if (bufferinfo.length != raw_img_size) {
        LOG(ERROR) << "bufferinfo.length = " << bufferinfo.length
                   << " data_ptr = " << reinterpret_cast<void*>(data_ptr);
      }
      if (counter < 4) {
        // skip the first few imgs because the time stamp data may not be
        // properly stamped in the very first imgs
      } else {
        V4l2BufferData buffer_and_data;
        buffer_and_data.counter = counter;
        buffer_and_data.img_data_ptr = img_data_buffers[counter % img_buffer_size];
        memcpy(&(buffer_and_data.img_data_ptr->at(0)), data_ptr, bufferinfo.length);
        VLOG(1) << "imgs_queue (" << imgs_queue.size() << ") pushing back";
        imgs_queue.push_back(buffer_and_data);
        VLOG(1) << "imgs_queue (" << imgs_queue.size() << ") pushed back";
      }
    } else {
      LOG(FATAL) << "Sensor type not supported" << FLAGS_sensor_type;
    }
    ++counter;
    usleep(1000);  // sleep for 1ms
    VLOG(1) << "========= thread_pull_img loop ends";
  }
  // Deactivate streaming at the end of the program
  VLOG(1) << "========= thread_pull_img stops";
}
#endif  // __linux__
void thread_draw_and_save_img(int fd) {
  VLOG(1) << "========= thread_draw_and_save_img thread starts";
  bool headless_os = FLAGS_headless;
#ifdef __linux__  // predefined by gcc
  const char* env_display_p = std::getenv("DISPLAY");
  if (env_display_p == nullptr) {
    std::cout << "You are running headless OS. No window will be shown" << std::endl;
    headless_os = true;
  }
#endif
  if (!headless_os) {
    cv::namedWindow("img_lr");
    cv::moveWindow("img_lr", 1, 1);
  }
  uint64_t last_img_time_100_us = 0;
  std::unique_ptr<XP_DRIVER::LI_SENSOR::Counter32To64> counter32To64_ptr;
  if (FLAGS_sensor_type == "LI") {
    counter32To64_ptr.reset(new XP_DRIVER::LI_SENSOR::Counter32To64(LI_CLOCK_32BIT_MAX_COUNT));
  } else if (FLAGS_sensor_type == "XP"
             || FLAGS_sensor_type == "XP2"
             || FLAGS_sensor_type == "XP3"
             || FLAGS_sensor_type == "UDP"
             || FLAGS_sensor_type == "OCV") {
    counter32To64_ptr.reset(new XP_DRIVER::LI_SENSOR::Counter32To64(XP_CLOCK_32BIT_MAX_COUNT));
  } else {
    LOG(FATAL) << "Sensor type not supported " << FLAGS_sensor_type;
  }
  // for debug
  uint64_t last_img_time_100_us_debug = 0;
  uint64_t last_img_count_wo_overflow_debug = 0;
  XP::DuoCalibParam calib_param;
  cv::Mat disparity_img;
  cv::Mat depth_canvas;
  cv::Mat hist_canvas;
  cv::Ptr<cv::StereoBM> bm_matcher = cv::StereoBM::create(96, /* numDisparities divisble by 16*/
                                                          21  /* blockSize */);
  if (!FLAGS_calib_yaml.empty()) {
    if (!calib_param.LoadCamCalibFromYaml(FLAGS_calib_yaml)) {
      LOG(ERROR) << FLAGS_calib_yaml << " cannot be loaded";
      return;
    }
    if (calib_param.Camera.img_size.width != 640 || calib_param.Camera.img_size.height != 480) {
      LOG(ERROR) << "calib_param.Camera.img_size " << calib_param.Camera.img_size;
      return;
    }
  }
  std::vector<cv::Matx34f> proj_mat_lr(2);
  if (FLAGS_calib_verify || FLAGS_orb_verify) {
    CHECK(!FLAGS_calib_yaml.empty());
    CHECK_EQ(calib_param.Camera.D_T_C_lr.size(), 2);
    for (int lr = 0; lr < 2; ++lr) {
      Eigen::Matrix4f C_T_W = calib_param.Camera.D_T_C_lr[lr].inverse();
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
          proj_mat_lr[lr](i, j) = C_T_W(i, j);
        }
      }
    }
  }
  // apirl tags
  XP::AprilTagDetector ap_detector;
  // udp
  int udp_sock_out = -1;
  struct sockaddr_in udp_addr_out;
  // Create a socket
  if (!FLAGS_udp_ip.empty()) {
    if ((udp_sock_out = socket(AF_INET , SOCK_DGRAM , 0)) < 0) {
      LOG(ERROR) << "Could not create socket";
    } else {
      // Prepare the sockaddr_in structure
      udp_addr_out.sin_family = AF_INET;
      udp_addr_out.sin_addr.s_addr = inet_addr(FLAGS_udp_ip.c_str());
      udp_addr_out.sin_port = htons(FLAGS_udp_port);
      std::cout << "UDP stream setup done. Sending data to "
                << FLAGS_udp_ip << " via " << FLAGS_udp_port << std::endl;
    }
  }

  // mode compatibility is done in main
  constexpr int row_num = 480;
  constexpr int col_num = 640;
  // Concatenate left and right image together for faster display on Odroid
  cv::Mat img_lr(row_num, col_num * 2, CV_8UC1);
  cv::Mat img_l = img_lr(cv::Rect(0, 0, col_num, row_num));
  cv::Mat img_r = img_lr(cv::Rect(col_num, 0, col_num, row_num));
  cv::Mat img_color_lr(row_num, col_num * 2, CV_8UC3);
  cv::Mat img_color_l = img_color_lr(cv::Rect(0, 0, col_num, row_num));
  cv::Mat img_color_r = img_color_lr(cv::Rect(col_num, 0, col_num, row_num));

  auto last_time = std::chrono::steady_clock::now();
  XP_DRIVER::XP_SENSOR::ImuReader imu_reader;  // to read the imu from img
  int frame_counter = 0;
  int zero_cnt = 0;
  uint64_t last_xp_imu_time = 0;
  // for ORB detector
  std::vector<cv::Mat_<uchar>> cam_mask_lr(2);
  if (!FLAGS_calib_yaml.empty()) {
    for (int lr = 0; lr < 2; ++lr) {
      float fov_deg;
      CHECK(XP::generate_cam_mask(calib_param.Camera.cv_camK_lr[lr],
                                  calib_param.Camera.cv_dist_coeff_lr[lr],
                                  calib_param.Camera.img_size,
                                  &cam_mask_lr[lr],
                                  &fov_deg));
      LOG(INFO) << "Generate cam " << lr << " mask (fov: " << fov_deg << " deg)";
    }
  }

  cv::BFMatcher orb_matcher(cv::NORM_HAMMING);
  while (run_flag) {
    VLOG(1) << "========= thread_draw_and_save_img loop starts";
    // check if the imgs queue is too long
    bool pop_to_back = FLAGS_calib_verify || FLAGS_orb_verify;
    if (imgs_queue.size() > 10) {
      pop_to_back = true;
      LOG(ERROR) << "imgs_queue too long (" << imgs_queue.size()
                 << "). Pop to back";
    }
    V4l2BufferData buffer_and_data;
    if (pop_to_back) {
      // record and calib_verify cannot be set at the same time
      if (!imgs_queue.wait_and_pop_to_back(&buffer_and_data)) {
        break;
      }
      VLOG(1) << "imgs_queue.wait_and_pop_front done";
    } else {
      // if recording is needed
      VLOG(1) << "imgs_queue.wait_and_pop_front...";
      if (!imgs_queue.wait_and_pop_front(&buffer_and_data)) {
        break;
      }
      VLOG(1) << "imgs_queue.wait_and_pop_front done img size "
                 << buffer_and_data.img_data_ptr->size();
    }
    // CHECK_EQ(buffer_and_data.img_data_ptr->size(), 640 * 480 * 2);

    // the first few image contains junk data
    if (buffer_and_data.counter < 10) {
      continue;
    }
    // read imu from the img data
    // Do not skip any imgs
    const int imu_data_len = 17;
    int imu_data_in_img_pos = 0;
    if (FLAGS_sensor_type == "OCV") {
      imu_data_in_img_pos = XP_DRIVER::XP_SENSOR::OpencvVideoCap::IMU_DATA_POS;
    }
    if (FLAGS_sensor_type == "XP"
        || FLAGS_sensor_type == "XP2"
        || FLAGS_sensor_type == "XP3"
        || FLAGS_sensor_type == "OCV"
        || FLAGS_sensor_type == "UDP") {
      if (!g_pull_imu) {
        // read imu from the image
        // com port is not stable in Linux

        uint8_t* imu_burst_data_pos =  &(buffer_and_data.img_data_ptr->at(imu_data_in_img_pos));
        uint32_t imu_num = 0;
        // Support different versions of firmware. Old version is 25 Hz,
        // and the new version is 500 Hz.
        if (*(imu_burst_data_pos + 16) == 0) {
          imu_num = 1;
        } else {
          imu_num = *(reinterpret_cast<uint32_t *>(imu_burst_data_pos + imu_data_len));
          // update burst imu data position.
          imu_burst_data_pos = imu_burst_data_pos + imu_data_len + 4;
        }
        XP_DRIVER::LI_SENSOR::XP_20608_data imu_data;
        for (int imu_i = 0; imu_i < imu_num; ++imu_i) {
          if (imu_reader.get_imu_from_img(imu_burst_data_pos + imu_i * imu_data_len, &imu_data)) {
            // imu_reader already skipped a few imu samples at the start
            if (FLAGS_sensor_type == "XP"
                || FLAGS_sensor_type == "XP2"
                || FLAGS_sensor_type == "XP3"
                || FLAGS_sensor_type == "OCV") {
              if (g_first_imu_clock_count == 0) {
                g_first_imu_clock_count = imu_reader.first_imu_clock_count();
                LOG(INFO) << "g_first_imu_clock_count = " << g_first_imu_clock_count;
              }
            } else if (FLAGS_sensor_type == "UDP") {
              g_first_imu_clock_count = 1;  // time has already been offset to 0 in server
            } else {
              LOG(FATAL) << "logic error";
            }
            imu_data.clock_count -= g_first_imu_clock_count * 10;
            g_last_imu_time = imu_data.clock_count;
            g_imu_rate = imu_reader.imu_rate();
            VLOG(1) << "Getting new IMU at time " << imu_data.clock_count;
            if (!imu_samples_queue.empty()) {
              int64_t imu_time_increment =
                  static_cast<int64_t>(imu_data.clock_count) -
                  static_cast<int64_t>(last_xp_imu_time);
              if (imu_time_increment > 1000) {  // unit is 100 us
                LOG(ERROR) << "Imu time jumps" << last_xp_imu_time
                          << " -> " << imu_data.clock_count;
              }
              if (imu_time_increment < 0) {
                LOG(FATAL) << "Imu time revert " << last_xp_imu_time
                          << " -> " << imu_data.clock_count;
              }
            }
            last_xp_imu_time = imu_data.clock_count;
            // after pushing back, the imu_data becomes undefined
            imu_samples_queue.push_back(imu_data);
          }
        }
      }
    }
    if (g_first_imu_clock_count == 0) {
      VLOG(1) << "========= thread_draw_and_save_img loop ends g_first_imu_clock_count = 0";
      continue;
    }
    // Get image timestamp so as to decide which frame to skip
    uint64_t clock_count_with_overflow = 0;
    std::vector<uint8_t>& img_data = *buffer_and_data.img_data_ptr;
    if (FLAGS_sensor_type == "LI") {
      // get time code from buffer_and_data.img_data
      clock_count_with_overflow = static_cast<uint64_t>(img_data[0]);
      clock_count_with_overflow |= static_cast<uint64_t>(img_data[1]) << 8;
      clock_count_with_overflow |= static_cast<uint64_t>(img_data[2]) << 16;
      // this produces wrong result if uint32_t is used
      clock_count_with_overflow |= static_cast<uint64_t>(img_data[3]) << 24;
      if (FLAGS_verbose) {
        cout << " clock_count_with_overflow " << clock_count_with_overflow
             << " pts "
             << static_cast<uint64_t>(img_data[0]) << " "
             << static_cast<uint64_t>(img_data[1]) << " "
             << static_cast<uint64_t>(img_data[2]) << " "
             << static_cast<uint64_t>(img_data[3]) << endl;
      }
    } else if (FLAGS_sensor_type == "XP"
               || FLAGS_sensor_type == "XP2"
               || FLAGS_sensor_type == "XP3"
               || FLAGS_sensor_type == "UDP") {
      clock_count_with_overflow =
          XP_DRIVER::XP_SENSOR::get_timestamp_in_img(&(img_data[0]));
    } else if (FLAGS_sensor_type == "OCV") {
      clock_count_with_overflow =
        XP_DRIVER::XP_SENSOR::get_timestamp_in_img(&(img_data[imu_data_in_img_pos]));
    } else {
      LOG(FATAL) << "Sensor type not supported " << FLAGS_sensor_type;
    }
    uint64_t clock_count_wo_overflow =
        counter32To64_ptr->convertNewCount32(clock_count_with_overflow);
    if (last_img_count_wo_overflow_debug > clock_count_wo_overflow) {
      LOG(FATAL) << "last_img_count_wo_overflow_debug > clock_count_wo_overflow "
                 << last_img_count_wo_overflow_debug
                 << " > " << clock_count_wo_overflow
                 << " overflow count " << counter32To64_ptr->getOverflowCount();
    }
    if (clock_count_wo_overflow < g_first_imu_clock_count) {
      // only happens at the beginning
      continue;
    }
    last_img_count_wo_overflow_debug = clock_count_wo_overflow;
    uint64_t img_time_100_us = 0;
    if (FLAGS_sensor_type == "LI") {
      img_time_100_us =
        (clock_count_wo_overflow - g_first_imu_clock_count) / (LI_BOARD_CLOCK_HZ_INT / 10000);
    } else if (FLAGS_sensor_type == "XP"
               || FLAGS_sensor_type == "XP2"
               || FLAGS_sensor_type == "XP3"
               || FLAGS_sensor_type == "OCV"
               || FLAGS_sensor_type == "UDP") {
      img_time_100_us =
        (clock_count_wo_overflow - g_first_imu_clock_count)  * 10;
    } else {
      LOG(FATAL) << "Sensor type not supported" << FLAGS_sensor_type;
    }
    // Get frame rate at ~20 Hz
    if (last_img_time_100_us != 0) {
      if (img_time_100_us < last_img_time_100_us) {
        LOG(ERROR) << "img_time_100_us " << img_time_100_us << " < "
                 << " last_img_time_100_us " << last_img_time_100_us;
      }
      // skip if gap is smaller than 40 ms
      // no skip is sensor is XP2 since XP2 sensor img rate is already lowered
      if (img_time_100_us - last_img_time_100_us < 400
          && FLAGS_sensor_type != "XP2" ) {
        VLOG(1) << "skip img_time_100_us - last_img_time_100_us < 400";
        continue;
      }
    }
    last_img_time_100_us = img_time_100_us;
    if (FLAGS_verbose) {
      if (FLAGS_sensor_type == "LI") {
        cout << " clock_count_wo_overflow " << clock_count_wo_overflow
             << " overflow_count " << counter32To64_ptr->getOverflowCount();
      } else if (FLAGS_sensor_type == "XP"
                 || FLAGS_sensor_type == "XP2"
                 || FLAGS_sensor_type == "XP3") {
        cout << " clock_count_wo_overflow " << clock_count_wo_overflow
             << " overflow_count " << counter32To64_ptr->getOverflowCount()
             << " pts "
             << static_cast<uint64_t>(img_data[12]) << " "
             << static_cast<uint64_t>(img_data[13]) << " "
             << static_cast<uint64_t>(img_data[14]) << " "
             << static_cast<uint64_t>(img_data[15]) << " "
             << static_cast<uint64_t>(img_data[16]) << endl;
      }
    }
    if (FLAGS_sensor_type == "LI") {
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; ++j) {
          img_l.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2 + 1];
          img_r.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2];
        }
      }
    } else if (FLAGS_sensor_type == "XP") {
#ifndef __ARM_NEON__
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; ++j) {
          img_l.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2];
          img_r.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2 + 1];
        }
      }
#else
      uint8_t* img_data_ptr = img_data.data();
      uint8_t* img_l_data_ptr = img_l.ptr<uint8_t>();
      uint8_t* img_r_data_ptr = img_r.ptr<uint8_t>();
      uint8x16x2_t tmp_data;
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; j += 16) {
          tmp_data = vld2q_u8(img_data_ptr);
          vst1q_u8(img_l_data_ptr, tmp_data.val[0]);
          vst1q_u8(img_r_data_ptr, tmp_data.val[1]);
          img_l_data_ptr += 16;
          img_r_data_ptr += 16;
          img_data_ptr += 32;
        }
      }
#endif
    } else if (FLAGS_sensor_type == "XP2") {
      uint8_t* img_data_ptr = img_data.data();
#ifndef __ARM_NEON__
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; ++j) {
          img_l.at<uint8_t>(row_num - i - 1, col_num - j - 1) = *(img_data_ptr + 0);
          img_r.at<uint8_t>(i, j) = *(img_data_ptr + 1);
          img_data_ptr += 2;
        }
      }
#else
      CHECK_EQ(col_num & 7, 0);
      img_data_ptr = img_data.data();
      for (int r = 0; r < row_num; ++r) {
        uint8_t* l_frame_ptr = img_lr.ptr<uint8_t>(row_num - r - 1) + col_num;
        uint8_t* r_frame_ptr = img_lr.ptr<uint8_t>(r) + col_num;
        for (int c = 0; c < col_num; c += 8) {
          uint8x8x2_t data = vld2_u8(img_data_ptr);
          l_frame_ptr -= 8;
          data.val[0] = vrev64_u8(data.val[0]);
          vst1_u8(r_frame_ptr, data.val[1]);
          vst1_u8(l_frame_ptr, data.val[0]);
          img_data_ptr += 16;
          r_frame_ptr += 8;
        }
      }
#endif
    } else if (FLAGS_sensor_type == "XP3") {
      uint8_t* img_data_ptr = img_data.data();
    #ifndef __ARM_NEON__
      zero_cnt = 0;
      for (int r = 0; r < row_num; ++r) {
        if (*(img_data_ptr + r * 2 * col_num - 1 ) == 0)
          zero_cnt++;
      }
      if (zero_cnt == row_num && frame_counter == 1) {
        std::cout << "Warning: image num [" << img_time_100_us
                  << "] found zero column." << std::endl;
      }
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; ++j) {
          img_l.at<uint8_t>(row_num - i - 1, col_num - j - 1) = *(img_data_ptr + 0);
          if (zero_cnt == row_num) {
            if ( j < col_num - 1)
              img_r.at<uint8_t>(i, j + 1) = *(img_data_ptr + 1);
            else
              img_r.at<uint8_t>(i, 0) = 0;
          } else {
            img_r.at<uint8_t>(i, j) = *(img_data_ptr + 1);
          }
          img_data_ptr += 2;
        }
      }
    #else
      // TODO(hongtian): add right eye shift one byte when detected zero column
      CHECK_EQ(col_num & 7, 0);
      img_data_ptr = img_data.data();
      for (int r = 0; r < row_num; ++r) {
        uint8_t* l_frame_ptr = img_lr.ptr<uint8_t>(row_num - r - 1) + col_num;
        uint8_t* r_frame_ptr = img_lr.ptr<uint8_t>(r) + col_num;
        for (int c = 0; c < col_num; c += 8) {
          uint8x8x2_t data = vld2_u8(img_data_ptr);
          l_frame_ptr -= 8;
          data.val[0] = vrev64_u8(data.val[0]);
          vst1_u8(r_frame_ptr, data.val[1]);
          vst1_u8(l_frame_ptr, data.val[0]);
          img_data_ptr += 16;
          r_frame_ptr += 8;
        }
      }
    #endif
    } else if (FLAGS_sensor_type == "UDP") {
      for (int i = 0; i < row_num; ++i) {
        for (int j = 0; j < col_num; ++j) {
          img_l.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2 + 1];
          img_r.at<uint8_t>(i, j) = img_data[i * col_num * 2 + j * 2];
        }
      }
    } else if (FLAGS_sensor_type == "OCV") {
      XP_DRIVER::XP_SENSOR::OpencvVideoCap::copy_to_lr(&img_l,
                                                       &img_r,
                                                       &(img_data[0]));
    } else {
      LOG(FATAL) << FLAGS_sensor_type;
    }
    int det_count_l = 0;
    int det_count_r = 0;
    int match_count = 0;
    int less_than_1_count = 0;
    int less_than_2_count = 0;
    std::vector<std::vector<cv::Point2f>> matched_raw_pnts(2);
    if (FLAGS_calib_verify) {
      std::vector<std::vector<cv::KeyPoint>> apt_lr(2);
      ap_detector.detect(img_l, &(apt_lr[0]));
      ap_detector.detect(img_r, &(apt_lr[1]));
      // Plot marker to img
      for (const auto& kp : apt_lr[0]) {
        cv::circle(img_l, kp.pt, 2, cv::Scalar(255, 255, 255));
      }
      for (const auto& kp : apt_lr[1]) {
        cv::circle(img_r, kp.pt, 2, cv::Scalar(255, 255, 255));
      }
      det_count_l = apt_lr[0].size();
      det_count_r = apt_lr[1].size();
      if (!apt_lr[0].empty() && !apt_lr[1].empty()) {
        matched_raw_pnts[0].reserve(apt_lr[0].size());
        matched_raw_pnts[1].reserve(apt_lr[1].size());
        for (size_t it_l = 0; it_l < apt_lr[0].size(); ++it_l) {
          CHECK_GE(apt_lr[0][it_l].class_id, 0);
          for (size_t it_r = 0; it_r < apt_lr[1].size(); ++it_r) {
            if (apt_lr[1][it_r].class_id == apt_lr[0][it_l].class_id) {
              // we got a matched april tag
              ++match_count;
              matched_raw_pnts[0].push_back(apt_lr[0][it_l].pt);
              matched_raw_pnts[1].push_back(apt_lr[1][it_r].pt);
              break;
            }
          }
        }
      }
    } else if (FLAGS_orb_verify) {
      // same as VioMapperBase::count_small_error_stereo_match
      std::vector<std::vector<cv::KeyPoint>> kp_lr(2);
      std::vector<cv::Mat> orb_lr(2);
      const int request_feat_num = 200;
      const int fast_thresh = 15;
      XP::detect_orb_features(img_l.clone(),  // ow, detect_orb_features uses img_lr as a whole
                              cam_mask_lr[0],
                              request_feat_num,
                              2,  // pyra_level,
                              fast_thresh,
                              true,  // use_fast (or TomasShi)
                              5,  // enforce_uniformatiy_radius (less than 5 means no enforcement)
                              &kp_lr[0],
                              &orb_lr[0]);
      det_count_l = kp_lr[0].size();
      XP::detect_orb_features(img_r.clone(),  // ow, detect_orb_features uses img_lr as a whole
                              cam_mask_lr[1],
                              request_feat_num,
                              2,  // pyra_level,
                              fast_thresh,
                              true,  // use_fast ( or TomasShi)
                              5,  // enforce_uniformatiy_radius (less than 5 means no enforcement)
                              &kp_lr[1],
                              &orb_lr[1]);
      det_count_r = kp_lr[1].size();
      if (!kp_lr[0].empty() && !kp_lr[1].empty()) {
        // matching
        std::vector<std::vector<std::vector<cv::DMatch>>> matches_lr(2);
        std::vector<cv::Mat> matching_mask_lr(2);
        matching_mask_lr[0].create(orb_lr[0].rows, orb_lr[1].rows, CV_8U);
        matching_mask_lr[0].setTo(0x01);
        matching_mask_lr[1].create(orb_lr[1].rows, orb_lr[0].rows, CV_8U);
        matching_mask_lr[1].setTo(0x01);
        orb_matcher.knnMatch(orb_lr[0], orb_lr[1], matches_lr[0], 1, matching_mask_lr[0]);
        orb_matcher.knnMatch(orb_lr[1], orb_lr[0], matches_lr[1], 1, matching_mask_lr[1]);
        // cross validation
        std::vector<std::vector<bool>> is_matched_lr(2);
        for (int  lr = 0; lr < 2; ++lr) {
          is_matched_lr[lr].resize(orb_lr[lr].rows, false);
          matched_raw_pnts[lr].reserve(orb_lr[lr].rows);
          for (auto& kp : kp_lr[lr]) {
            kp.class_id = 0;
          }
        }
        for (int it_pnt_0 = 0; it_pnt_0 < orb_lr[0].rows; ++it_pnt_0) {
          const int match_id_in_1 = matches_lr[0][it_pnt_0][0].trainIdx;
          const int match_id_in_0 = matches_lr[1][match_id_in_1][0].trainIdx;
          if (match_id_in_0 == it_pnt_0) {
            // Potential bug here as it_pnt_0 can be 0, which is later on considered as a no-match.
            // todo(bao) assign a unique shared class_id to kp_lr starting from 1.
            kp_lr[0][it_pnt_0].class_id = match_id_in_1;
            kp_lr[1][match_id_in_1].class_id = it_pnt_0;
            ++match_count;
            matched_raw_pnts[0].push_back(kp_lr[0][it_pnt_0].pt);
            matched_raw_pnts[1].push_back(kp_lr[1][match_id_in_1].pt);
          }
        }
        // Plot marker to img
        for (const auto& kp : kp_lr[0]) {
          if (kp.class_id > 0) {
            cv::circle(img_l, kp.pt, 4, cv::Scalar(255, 255, 255));
          } else {
            cv::circle(img_l, kp.pt, 2, cv::Scalar(255, 255, 255));
          }
        }
        for (const auto& kp : kp_lr[1]) {
          if (kp.class_id > 0) {
            cv::circle(img_r, kp.pt, 4, cv::Scalar(255, 255, 255));
          } else {
            cv::circle(img_r, kp.pt, 2, cv::Scalar(255, 255, 255));
          }
        }
      }
    }
    if (match_count != 0) {
      CHECK_EQ(matched_raw_pnts[0].size(), match_count);
      CHECK_EQ(matched_raw_pnts[1].size(), match_count);
      std::vector<std::vector<cv::Point2f>> matched_pnts(2);
      for (int lr = 0; lr < 2; ++lr) {
        cv::undistortPoints(matched_raw_pnts[lr],
                            matched_pnts[lr],
                            calib_param.Camera.cv_camK_lr[lr],
                            calib_param.Camera.cv_dist_coeff_lr[lr]);
      }

      cv::Mat homo_pnts_3d;
      cv::triangulatePoints(proj_mat_lr[0], proj_mat_lr[1],
                            matched_pnts[0], matched_pnts[1], homo_pnts_3d);
      CHECK_EQ(homo_pnts_3d.type(), CV_32F);
      CHECK_EQ(homo_pnts_3d.rows, 4);
      CHECK_EQ(homo_pnts_3d.cols, match_count);
      std::vector<cv::Point3f> pnts3d(match_count);
      for (int i = 0; i < match_count; ++i) {
        pnts3d[i].x = homo_pnts_3d.at<float>(0, i) / homo_pnts_3d.at<float>(3, i);
        pnts3d[i].y = homo_pnts_3d.at<float>(1, i) / homo_pnts_3d.at<float>(3, i);
        pnts3d[i].z = homo_pnts_3d.at<float>(2, i) / homo_pnts_3d.at<float>(3, i);
      }
      std::vector<cv::Point2f> projs_l;
      cv::projectPoints(pnts3d,
                        cv::Matx31d::zeros(),
                        cv::Matx31d::zeros(),
                        calib_param.Camera.cv_camK_lr[0],
                        calib_param.Camera.cv_dist_coeff_lr[0],
                        projs_l);
      CHECK_EQ(projs_l.size(), match_count);
      CHECK_EQ(matched_pnts[0].size(), match_count);
      for (int i = 0; i < match_count; ++i) {
        if (pnts3d[i].z > 0) {
          const float diff_x = projs_l[i].x - matched_raw_pnts[0][i].x;
          const float diff_y = projs_l[i].y - matched_raw_pnts[0][i].y;
          if (diff_x * diff_x + diff_y * diff_y < 1) {
            ++less_than_1_count;
            ++less_than_2_count;
            cv::circle(img_l, matched_raw_pnts[0][i], 8, cv::Scalar(255, 255, 255));
          } else if (diff_x * diff_x + diff_y * diff_y < 2) {
            ++less_than_2_count;
          } else {
            if (FLAGS_calib_verify) {
              cv::line(img_l, projs_l[i], matched_raw_pnts[0][i], cv::Scalar(255, 255, 255));
            } else {
              // there must some matching false alarms.
              // don't show obviously wrong matches
              if (diff_x * diff_x + diff_y * diff_y < 900) {
                cv::line(img_l, projs_l[i], matched_raw_pnts[0][i], cv::Scalar(255, 255, 255));
              }
            }
          }
        }
      }
    }
    if (FLAGS_undistort) {
      cv::Mat img_undistorted(row_num, col_num, CV_8UC1);
      cv::remap(img_l, img_undistorted,
                calib_param.Camera.undistort_map_op1_lr[0],
                calib_param.Camera.undistort_map_op2_lr[0], cv::INTER_LINEAR);
      img_undistorted.copyTo(img_l);
      cv::remap(img_r, img_undistorted,
                calib_param.Camera.undistort_map_op1_lr[1],
                calib_param.Camera.undistort_map_op2_lr[1], cv::INTER_LINEAR);
      img_undistorted.copyTo(img_r);
      bm_matcher->compute(img_l, img_r, disparity_img);
      if (!headless_os) {
        if (depth_canvas.rows == 0) {
          depth_canvas.create(disparity_img.size(), CV_8UC3);
        }
        for (int i = 0; i < disparity_img.rows; ++i) {
          for (int j = 0; j < disparity_img.cols; ++j) {
            depth_canvas.at<cv::Vec3b>(i, j) = XP::depth16S2color(disparity_img.at<int16_t>(i, j));
          }
        }
      }
    }
    if (FLAGS_show_horizontal_line) {
      for (int i = 20; i < img_l.rows; i+= 20) {
        for (int j = 0; j < img_l.cols; ++j) {
          img_l.at<uchar>(i, j) = 255;
          img_r.at<uchar>(i, j) = 255;
        }
      }
    }
    if (!headless_os && FLAGS_show_hist) {
      std::vector<int> histogram;
      if (XP::sampleBrightnessHistogram(img_l, &histogram)) {
        XP::drawHistogram(&hist_canvas, histogram);
        imshow("histogram", hist_canvas);
      }
    }
#ifdef __linux__  // predefined by gcc
    if (g_auto_gain) {
      if (frame_counter % 5 == 3) {
        // change auto gain every 5 frames
        // compute auto gain exposure table index
        const bool verbose = false;
        if (FLAGS_old_aec_method) {
          // The old aec algorithm
          if (XP::computeNewGainAndExposure(img_l, &g_gain_val, &g_exp_val)) {
            XP_DRIVER::XP_SENSOR::set_exp_percentage(fd, g_exp_val, verbose);
            XP_DRIVER::XP_SENSOR::set_gain_percentage(fd, g_gain_val, verbose);
          } else {
            LOG(ERROR) << "computeNewGainAndExposure fails";
          }
        } else {
          if (XP::computeNewAecTableIndex(img_l, &g_aec_index)) {
            XP_DRIVER::XP_SENSOR::set_aec_index(fd, g_aec_index, verbose);
          } else {
            LOG(ERROR) << "computeNewAecTableIndex fails";
          }
        }
      }
    }
#endif  // __linux__
    if (depth_canvas.rows != 0 && !headless_os) {
      imshow("depth_canvas", depth_canvas);
    }
    bool save_img = !FLAGS_spacebar_mode;
    char keypressed = cv::waitKey(10);
    if (keypressed == 27) {
      // ESC
      imu_samples_queue.kill();
      run_flag = false;
      break;
    } else if (keypressed == 32) {
      // space
      save_img = true;
    }
#ifdef __linux__  // predefined by gcc
    if (FLAGS_sensor_type == "XP"
        || FLAGS_sensor_type == "XP2"
        || FLAGS_sensor_type == "XP3") {
      if (!g_auto_gain) {
        // keyboard control for gain/exp percentage
        bool update_gain_exp_percentage = true;
        if (keypressed == '1') {
          g_gain_val = 1;
          g_exp_val = 1;
        } else if (keypressed == '2') {
          g_gain_val = 30;
          g_exp_val = 30;
        } else if (keypressed == '3') {
          g_gain_val = 60;
          g_exp_val = 60;
        } else if (keypressed == '4') {
          g_gain_val = 100;
          g_exp_val = 100;
        } else if (keypressed == '+' || keypressed == '=') {
          g_exp_val = g_gain_val;
          g_gain_val += 5;
          g_exp_val += 5;
          if (g_gain_val > 100) g_gain_val = 100;
          if (g_exp_val > 100) g_exp_val = 100;
        } else if (keypressed == '-') {
          g_exp_val = g_gain_val;
          g_gain_val -= 5;
          g_exp_val -= 5;
          if (g_gain_val < 1) g_gain_val = 1;
          if (g_exp_val < 1) g_exp_val = 1;
        } else if (keypressed == 83) {  // right arrow
          g_exp_val = g_gain_val;
          if (g_gain_val < 100) {
            g_gain_val += 1;
            g_exp_val += 1;
          }
        } else if (keypressed == 81) {  // left arrow
          g_exp_val = g_gain_val;
          if (g_gain_val > 0) {
            g_gain_val -= 1;
            g_exp_val -= 1;
          }
        } else if (keypressed == 82) {  // up arrow
          g_gain_val += 10;
          if (g_gain_val > 100) {
            g_gain_val = 100;
          }
          g_exp_val = g_gain_val;
        } else if (keypressed == 84) {  // down arrow
          if (g_gain_val >= 10) {
            g_gain_val -= 10;
          } else {
            g_gain_val = 0;
          }
          g_exp_val = g_gain_val;
        } else {
          update_gain_exp_percentage = false;
        }

        // keyboard control for aec index
        using XP_DRIVER::XP_SENSOR::kAEC_steps;
        bool update_aec_index = true;
        if (keypressed == '[') {
          g_aec_index -= 5;
          if (g_aec_index < 0) g_aec_index = 0;
        } else if (keypressed == ']') {
          g_aec_index += 5;
          if (g_aec_index >= kAEC_steps) g_aec_index = kAEC_steps -1;
        } else if (keypressed == ',') {
          --g_aec_index;
          if (g_aec_index < 0) g_aec_index = 0;
        } else if (keypressed == '.') {
          ++g_aec_index;
          if (g_aec_index >= kAEC_steps) g_aec_index = kAEC_steps -1;
        } else {
          update_aec_index = false;
        }

        // update_aec_index has higher priority over update_gain_exp_percentage
        if (update_aec_index) {
          XP_DRIVER::XP_SENSOR::set_aec_index(fd, g_aec_index, true/*verbose*/);
        } else if (update_gain_exp_percentage) {
          std::cout << "gain/exp (%%)= " << g_gain_val << "/" << g_exp_val << "\n";
          XP_DRIVER::XP_SENSOR::set_exp_percentage(fd, g_gain_val, true/*verbose*/);
          XP_DRIVER::XP_SENSOR::set_gain_percentage(fd, g_exp_val, true/*verbose*/);
        }
      }
      if (keypressed == 'a' || keypressed == 'A') {
        g_auto_gain = !g_auto_gain;
      }
    }
#endif  // __linux__
    // debug
    if (last_img_time_100_us_debug != 0) {
      if (img_time_100_us < last_img_time_100_us_debug) {
        LOG(ERROR) << "img_time_100_us < last_img_time_100_us_debug "
                   << img_time_100_us << " < " << last_img_time_100_us_debug
                   << " clock_count_with_overflow " << clock_count_with_overflow
                   << " clock_count_wo_overflow " << clock_count_wo_overflow
                   << " counter32To64_ptr->getLastCounter32 "
                   << counter32To64_ptr->getLastCounter32()
                   << " counter32To64_ptr->getOverflowCount "
                   << counter32To64_ptr->getOverflowCount();
      }
      // check if consecutive frames time stamp is close
      if (img_time_100_us - last_img_time_100_us_debug > 1000) {
        LOG(ERROR) << "Img timestamp jumps. img_time_100_us " << img_time_100_us
                   << " last_img_time_100_us_debug " << last_img_time_100_us_debug
                   << " clock_count_with_overflow " << clock_count_with_overflow
                   << " clock_count_wo_overflow " << clock_count_wo_overflow
                   << " g_first_imu_clock_count " << g_first_imu_clock_count
                   << " overflow_count " << counter32To64_ptr->getOverflowCount();
      }
    }
    last_img_time_100_us_debug = img_time_100_us;
    if (udp_sock_out >= 0) {
      UdpImg udp_img;
      // downsample imgs
      for (int i = 0; i < 96; ++i) {
        for (int j = 0; j < 128; ++j) {
          udp_img.img_l_128x96[i * 128 + j] = img_l.at<uchar>(i * 5, j * 5);
          udp_img.img_r_128x96[i * 128 + j] = img_r.at<uchar>(i * 5, j * 5);
        }
      }
      // copy header
      memcpy(udp_img.img_l_128x96, &(img_data[0]), 20);
      // set time
      udp_img.img_time = img_time_100_us / 1e4;
      if (sendto(udp_sock_out, &udp_img, sizeof(udp_img), 0,
                 (struct sockaddr *)&udp_addr_out, sizeof(udp_addr_out)) < 0) {
        LOG(ERROR) << "sendto() failed -> " << inet_ntoa(udp_addr_out.sin_addr)
                   << ":" << ntohs(udp_addr_out.sin_port);
      }
    }
    // show some debug info
    auto time_after = std::chrono::steady_clock::now();
    int ms_passed =
      std::chrono::duration_cast<std::chrono::milliseconds>(time_after - last_time).count();
    last_time = time_after;
    std::string debug_string =
        "img rate " + boost::lexical_cast<std::string>(1000 / ms_passed) + " Hz"
        + " sec " + boost::lexical_cast<std::string>(img_time_100_us / 10000.f)
        + " imu rate " + boost::lexical_cast<std::string>(g_imu_rate)
        + " Hz time " + boost::lexical_cast<std::string>(g_last_imu_time / 10000.f);
    if (!headless_os) {
      // convert to color
      if (FLAGS_sensor_type == "XP3") {
        cv::cvtColor(img_l, img_color_l, cv::COLOR_BayerGB2BGR);
        cv::cvtColor(img_r, img_color_r, cv::COLOR_BayerGR2BGR);
      } else {
        cv::cvtColor(img_l, img_color_l, cv::COLOR_GRAY2BGR);
        cv::cvtColor(img_r, img_color_r, cv::COLOR_GRAY2BGR);
      }
      cv::putText(img_color_lr, debug_string,
        cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
      if (FLAGS_calib_verify || FLAGS_orb_verify) {
        std::string score = "N/A";
        if (match_count > 0) {
          if (FLAGS_calib_verify) {
            score = boost::lexical_cast<std::string>(100 * less_than_1_count / match_count);
          } else {
            // the guess is that there should at least 20 features that are good match
            if (less_than_1_count > 20) {
              score = "100";
            } else {
              score = boost::lexical_cast<std::string>(
                  100 * less_than_1_count / std::max(30, match_count));
            }
          }
        }
        cv::putText(img_color_lr, "score " + score
                    + " reproj < 1 (2) " + boost::lexical_cast<std::string>(less_than_1_count)
                    + " (" + boost::lexical_cast<std::string>(less_than_2_count)
                    + ") match # " + boost::lexical_cast<std::string>(match_count)
                    + " det " + boost::lexical_cast<std::string>(det_count_l)
                    + " / " + boost::lexical_cast<std::string>(det_count_r),
                    cv::Point(15, 35), cv::FONT_HERSHEY_COMPLEX, 0.5,
                              cv::Scalar(255, 0, 255), 1);
      }
#ifdef __ARM_NEON__
      if ((frame_counter & 1) == 0) {
        imshow("img_lr", img_color_lr);
      }
#else
      imshow("img_lr", img_color_lr);
#endif
    } else {
      std::cout << debug_string << std::endl;
    }
    if (save_img) {
      std::ostringstream ss;
      ss << std::setfill('0') << std::setw(10) << img_time_100_us;
      time_t tt = time(NULL);
      ss << "_" << tt;
      std::string timestamp = ss.str();
      if (zero_cnt == row_num && frame_counter == 1) {
        std::cout << "warning: zero column image num [" << timestamp
                  << "] will be saved." << std::endl;
      }
      cv::imwrite(FLAGS_record_path + "/l/" + timestamp + ".png", img_color_l);
      cv::imwrite(FLAGS_record_path + "/r/" + timestamp + ".png", img_color_r);

      if (FLAGS_save_image_bin == true) {
        /* store image in file */
        std::ofstream img_l_fstream;
        img_l_fstream.open((FLAGS_record_path + "/l/" + timestamp +".txt").c_str(),
                           std::iostream::trunc);
        if (img_l_fstream.is_open()) {
          cout << "write to " << FLAGS_record_path + "/img_data.txt " << endl;
        } else {
          cout << "Fail to open " << FLAGS_record_path + "/img_data.txt " << endl;
        }
        if (img_l_fstream.is_open()) {
          for (int i = 0; i < row_num; ++i) {
            for (int j = 0; j < col_num; ++j) {
              float pixe_data = img_l.at<uint8_t>(i, j);
              img_l_fstream << pixe_data << " ";
            }
            img_l_fstream << endl;
          }
        }
        img_l_fstream.flush();
        if (img_l_fstream.is_open()) {
          img_l_fstream.close();
        }
        std::ofstream img_r_fstream;
        img_r_fstream.open((FLAGS_record_path + "/r/" + timestamp +".txt").c_str(),
                            std::iostream::trunc);
        if (img_r_fstream.is_open()) {
          cout << "write to " << FLAGS_record_path + "/img_data.txt " << endl;
        } else {
          cout << "Fail to open " << FLAGS_record_path + "/img_data.txt " << endl;
        }
        if (img_r_fstream.is_open()) {
          for (int i = 0; i < row_num; ++i) {
            for (int j = 0; j < col_num; ++j) {
              float pixe_data = img_r.at<uint8_t>(i, j);
              img_r_fstream << pixe_data << " ";
            }
            img_r_fstream << endl;
          }
        }
        img_r_fstream.flush();
        if (img_r_fstream.is_open()) {
          img_r_fstream.close();
        }
      }
    }
    ++frame_counter;
    // waitKey will be used in the next loop
    usleep(1000);  // sleep for 1ms
    VLOG(1) << "========= thread_draw_and_save_img loop ends";
  }
  VLOG(1) << "========= thread_draw_and_save_img stops";
}  // NOLINT
#ifdef __linux__  // predefined by gcc
void thread_pull_LI_imu(int fd) {
  using namespace std::chrono;  // NOLINT
  XP_DRIVER::LI_SENSOR::Counter32To64 counter32To64(LI_CLOCK_32BIT_MAX_COUNT);
  // for debug
  uint64_t last_clock_count_64 = 0;
  while (run_flag) {
    // sleep for 9900 us to get ~100 Hz rate
    std::this_thread::sleep_for(std::chrono::microseconds(9900));
    XP_DRIVER::LI_SENSOR::XP_20608_data imu_data;
    const int start_pull_time_ms =
        duration_cast<microseconds>(steady_clock::now() - main_start_time).count();
    struct timespec start_pull_timespec;
    clock_gettime(CLOCK_MONOTONIC, &start_pull_timespec);
    if (XP_DRIVER::LI_SENSOR::IMU_DataAccess(fd, &imu_data)) {
      const int end_pull_time_ms =
          duration_cast<microseconds>(steady_clock::now() - main_start_time).count();
      struct timespec end_pull_timespec;
      clock_gettime(CLOCK_MONOTONIC, &end_pull_timespec);
      // alter imu time with sys time
      const int start_pull_us_since_main =
          (start_pull_timespec.tv_sec - main_start_timespec.tv_sec) * 1000000 +
          (start_pull_timespec.tv_nsec - main_start_timespec.tv_nsec) / 1000;
      const int end_pull_us_since_main =
          (end_pull_timespec.tv_sec - main_start_timespec.tv_sec) * 1000000 +
          (end_pull_timespec.tv_nsec - main_start_timespec.tv_nsec) / 1000;
      // imu_data.clock_count = (start_pull_us_since_main + end_pull_us_since_main) / 2;
      // imu_data.clock_count is uint32_t
      uint64_t clock_count_wo_overflow = counter32To64.convertNewCount32(imu_data.clock_count);
      if (g_first_imu_clock_count == 0) {
        g_first_imu_clock_count = clock_count_wo_overflow;
      }
      // only for debug
      if (last_clock_count_64 > 0) {
        if (clock_count_wo_overflow < last_clock_count_64) {
          LOG(FATAL) << " clock_count_wo_overflow " << clock_count_wo_overflow
                  << " < last_clock_count_64 " << last_clock_count_64;
        }
        if (clock_count_wo_overflow - last_clock_count_64 > LI_BOARD_CLOCK_HZ_INT / 10) {
          LOG(ERROR) << "IMU clock jumps. clock_count_64 " << clock_count_wo_overflow
          << " last_clock_count_64 " << last_clock_count_64;
        }
      }
      last_clock_count_64 = clock_count_wo_overflow;
      if (FLAGS_verbose) {
        cout << "imu data accel_gyro_temp_t=["
             << imu_data.accel[0] << ", " << imu_data.accel[1] << ", " << imu_data.accel[2] << ", "
             << imu_data.gyro[0] << ", " << imu_data.gyro[1] << ", " << imu_data.gyro[2] << ", "
             << imu_data.temp << ", " << imu_data.clock_count << ", "
             << clock_count_wo_overflow << ", "
             << clock_count_wo_overflow / LI_BOARD_CLOCK_HZ << "]"
             << " start_end_pull_time=[" << start_pull_time_ms << ", " << end_pull_time_ms << "]"
             << " start_end_pull_us_since_main=[" << start_pull_us_since_main << ", "
             << end_pull_us_since_main << "]" << endl;
      }
      uint64_t time_100us =
          (clock_count_wo_overflow - g_first_imu_clock_count) / (LI_BOARD_CLOCK_HZ_INT / 10000);
      g_last_imu_time = time_100us;
      imu_data.clock_count = time_100us;
      imu_samples_queue.push_back(imu_data);
      if (g_imu_sample_count < 0) {
        // init
        g_imu_sample_start_tp = std::chrono::steady_clock::now();
      }
      ++g_imu_sample_count;
      if (g_imu_sample_count > 20) {
        const int time_us = std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - g_imu_sample_start_tp).count();
        g_imu_rate = 1000000 / (time_us / g_imu_sample_count);
        g_imu_sample_start_tp = std::chrono::steady_clock::now();
        g_imu_sample_count = 0;
      }
    }
  }
  VLOG(1) << "========= thread_pull_LI_imu stops";
}
void thread_pull_XP_imu(int fd) {
  using namespace std::chrono;  // NOLINT
  XP_DRIVER::LI_SENSOR::Counter32To64 counter32To64(XP_CLOCK_32BIT_MAX_COUNT);
  // for debug
  uint64_t last_clock_count_64 = 0;
  while (run_flag && g_pull_imu) {
    // sleep for 9900 us to get ~100 Hz rate
    std::this_thread::sleep_for(std::chrono::microseconds(9900));
    XP_DRIVER::LI_SENSOR::XP_20608_data imu_data;
    if (XP_DRIVER::XP_SENSOR::IMU_DataAccess(fd, &imu_data)) {
      uint64_t clock_count_wo_overflow = counter32To64.convertNewCount32(imu_data.clock_count);
      if (g_first_imu_clock_count == 0) {
        g_first_imu_clock_count = clock_count_wo_overflow;
      }
      if (last_clock_count_64 > 0) {
        // only for debug
        if (clock_count_wo_overflow < last_clock_count_64) {
          LOG(FATAL) << " clock_count_wo_overflow " << clock_count_wo_overflow
                  << " < last_clock_count_64 " << last_clock_count_64;
        }
        if (clock_count_wo_overflow - last_clock_count_64 > LI_BOARD_CLOCK_HZ_INT / 10) {
          LOG(ERROR) << "IMU clock jumps. clock_count_64 " << clock_count_wo_overflow
          << " last_clock_count_64 " << last_clock_count_64;
        }
        if (last_clock_count_64 == clock_count_wo_overflow) {
          // we got a replicated IMU data. Reading time is too high.
          continue;
        }
      }
      last_clock_count_64 = clock_count_wo_overflow;
      if (FLAGS_verbose) {
        cout << "imu data accel_gyro_temp_t=["
             << imu_data.accel[0] << ", " << imu_data.accel[1] << ", " << imu_data.accel[2] << ", "
             << imu_data.gyro[0] << ", " << imu_data.gyro[1] << ", " << imu_data.gyro[2] << ", "
             << imu_data.temp << ", " << imu_data.clock_count << ", "
             << clock_count_wo_overflow << ", "
             << clock_count_wo_overflow / LI_BOARD_CLOCK_HZ << "]" << endl;
      }
      // convert to 100us
      uint64_t time_100us =
          (clock_count_wo_overflow - g_first_imu_clock_count) * 10;
      g_last_imu_time = time_100us;
      imu_data.clock_count = time_100us;
      imu_samples_queue.push_back(imu_data);
      if (g_imu_sample_count < 0) {
        // init
        g_imu_sample_start_tp = std::chrono::steady_clock::now();
      }
      ++g_imu_sample_count;
      if (g_imu_sample_count > 20) {
        const int time_us = std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - g_imu_sample_start_tp).count();
        g_imu_rate = 1000000 / (time_us / g_imu_sample_count);
        g_imu_sample_start_tp = std::chrono::steady_clock::now();
        g_imu_sample_count = 0;
      }
    } else {
      LOG(ERROR) << "You are using an old firmware. Pull IMU mode is disabled.";
      g_pull_imu = false;
    }
  }
  VLOG(1) << "========= thread_pull_XP_imu stops";
}
#endif  // __linux__

void thread_stream_opencv_camera(int cam_id) {
  XP_DRIVER::XP_SENSOR::OpencvVideoCap cap(cam_id);
  int frame_counter = 0;
  while (run_flag) {
    V4l2BufferData buffer_and_data;
    buffer_and_data.counter = frame_counter;
    cap.retrive(buffer_and_data.img_data_ptr);
    imgs_queue.push_back(buffer_and_data);
    ++frame_counter;
  }
}
void thread_write_imu_data() {
  VLOG(1) << "========= thread_write_imu_data thread starts";
  // write imu data
  std::ofstream imu_fstream;
  if (!FLAGS_record_path.empty()) {
    imu_fstream.open((FLAGS_record_path + "/imu_data.txt").c_str(), std::iostream::trunc);
    if (imu_fstream.is_open()) {
      cout << "write to " << FLAGS_record_path + "/imu_data.txt " << endl;
    } else {
      cout << "Fail to open " << FLAGS_record_path + "/imu_data.txt " << endl;
    }
  }
  while (run_flag) {
    XP_DRIVER::LI_SENSOR::XP_20608_data imu;
    if (!imu_samples_queue.wait_and_pop_front(&imu)) {
      break;
    }
    if (imu_fstream.is_open()) {
      imu.temp = 999;  // a faked val
      // the unit is log file is every 100us
      // imu.clock_count is in 100us
      if (FLAGS_sensor_type == "LI") {
        imu_fstream
            << imu.clock_count << " "
            << - imu.accel[1] << " "
            << - imu.accel[0] << " "
            << - imu.accel[2] << " "
            << - imu.gyro[1] / 180.f * M_PI << " "
            << - imu.gyro[0] / 180.f * M_PI << " "
            << - imu.gyro[2] / 180.f * M_PI << " "
            << imu.temp << endl;
      } else if (FLAGS_sensor_type == "XP"
                 || FLAGS_sensor_type == "OCV") {
        imu_fstream
            << imu.clock_count << " "
            << imu.accel[0] << " "
            << imu.accel[1] << " "
            << imu.accel[2] << " "
            << imu.gyro[0] / 180.f * M_PI << " "
            << imu.gyro[1] / 180.f * M_PI << " "
            << imu.gyro[2] / 180.f * M_PI << " "
            << imu.temp << endl;
      } else if (FLAGS_sensor_type == "XP2"
                 || FLAGS_sensor_type == "XP3") {
        imu_fstream
           << imu.clock_count << " "
           << - imu.accel[0] << " "
           << - imu.accel[1] << " "
           << imu.accel[2] << " "
           << - imu.gyro[0] / 180.f * M_PI << " "
           << - imu.gyro[1] / 180.f * M_PI << " "
           << imu.gyro[2] / 180.f * M_PI << " "
           << imu.temp << endl;
      } else {
        LOG(FATAL) << FLAGS_sensor_type;
      }
    }
  }
  if (imu_fstream.is_open()) {
    imu_fstream.close();
  }
  VLOG(1) << "========= thread_write_imu_data thread ends";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

#ifdef __ARM_NEON__
  if (FLAGS_cpu_core >= 0 && FLAGS_cpu_core < 8) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(FLAGS_cpu_core, &set);

    if (0 != sched_setaffinity(getpid(), sizeof(cpu_set_t), &set))
      exit(1);
    std::cout << "RUN ON CORE [" << FLAGS_cpu_core << "]" << std::endl;
  }
#endif  // __ARM_NEON__

  if (FLAGS_undistort && FLAGS_calib_yaml.empty()) {
    LOG(ERROR) << "You must load calib_txt to enable undistort";
    return -1;
  }
  if (FLAGS_calib_verify && FLAGS_calib_yaml.empty()) {
    LOG(ERROR) << "You must load calib_txt to enable calib_verify";
    return -1;
  }
  if (FLAGS_orb_verify && FLAGS_calib_yaml.empty()) {
    LOG(ERROR) << "You must load calib_txt to enable orb_verify";
    return -1;
  }
  if (FLAGS_orb_verify && FLAGS_undistort) {
    LOG(ERROR) << "orb_verify and undistort is set at the same time";
    return -1;
  }
  if (FLAGS_orb_verify && !FLAGS_record_path.empty()) {
    LOG(ERROR) << "orb_verify and record_path is set at the same time";
    return -1;
  }
  if (FLAGS_calib_verify && FLAGS_undistort) {
    LOG(ERROR) << "calib_verify and undistort is set at the same time";
    return -1;
  }
  if (FLAGS_calib_verify && !FLAGS_record_path.empty()) {
    LOG(ERROR) << "calib_verify and record_path is set at the same time";
    return -1;
  }
  if (FLAGS_orb_verify && FLAGS_calib_verify) {
    LOG(ERROR) << "orb_verify and calib_verify is set at the same time";
    return -1;
  }

  if (!FLAGS_record_path.empty()) {
    // First make sure record_path exist
    namespace fs = boost::filesystem;
    fs::path rec_path(FLAGS_record_path);
    if (fs::create_directories(rec_path)) {
      cout << "Created " << FLAGS_record_path << "\n";
    }

    // If record_path already exists, we will append time at the end of
    // the record path and hopefully it will have no collision, except the special
    // case of spacebar_mode, as we may intend to continue saving images in the same
    // record path.
    fs::path imu_data_file(rec_path / "imu_data.txt");
    if (fs::is_regular_file(imu_data_file) && !FLAGS_spacebar_mode) {
      std::cout << "Found existing recording files at " << FLAGS_record_path << "\n";
      std::time_t t = std::time(NULL);
      char buf[32];
      std::strftime(buf, sizeof(buf), "_%H%M%S", std::localtime(&t));
      FLAGS_record_path += std::string(buf);
      std::cout << "Rename record path to " << FLAGS_record_path << "\n";
      rec_path = fs::path(FLAGS_record_path);
      if (fs::create_directories(rec_path)) {
        cout << "Created " << FLAGS_record_path << "\n";
      }
    }
    fs::path imgs_l_path(rec_path / "l");
    fs::path imgs_r_path(rec_path / "r");
    fs::create_directory(imgs_l_path);
    fs::create_directory(imgs_r_path);
  }
#ifdef __linux__  // predefined by gcc
  struct v4l2_buffer bufferinfo;
#endif  // __linux__
  int device_file_id = 0;
  g_first_imu_clock_count = 0;
  g_imu_rate = 0;
  g_imu_sample_count = -1;  // for proper init
  g_last_imu_time = 0;
  g_auto_gain = false;
  run_flag = true;
  // For XP sensor, exp & gain will be overwritten by calling set_registers_to_default
  g_exp_val = 0x0080;
  g_gain_val = 16;
  g_aec_index = 80;
  if (FLAGS_sensor_type == "LI"
      || FLAGS_sensor_type == "XP"
      || FLAGS_sensor_type == "XP2"
      || FLAGS_sensor_type == "XP3") {
#ifdef __linux__  // predefined by gcc
    if (!XP_DRIVER::LI_SENSOR::init_v4l2(FLAGS_dev_id, &device_file_id, &bufferinfo)) {
      LOG(ERROR) << FLAGS_dev_id << " cannot be init";
      // try to turn stream off
      XP_DRIVER::LI_SENSOR::stop_v4l2(&device_file_id, bufferinfo);
      exit(-1);
    }
    if (FLAGS_sensor_type == "XP"
        || FLAGS_sensor_type == "XP2"
        || FLAGS_sensor_type == "XP3") {
      // enable or disable imu embed img funciton of firmware
      XP_DRIVER::XP_SENSOR::xp_imu_embed_img(device_file_id, FLAGS_imu_from_image);

      constexpr bool verbose = false;  // Do NOT turn verbose on if not using the latest firmware
      XP_DRIVER::XP_SENSOR::set_registers_to_default(device_file_id, g_aec_index, verbose,
                                                     &g_exp_val, &g_gain_val);
    }
    VLOG(1) << "init_v4l2 success!";
#else  // __linux__
    LOG(ERROR) << "Only linux supports live sensor mode";
    return -1;
#endif  // __linux__
  } else if (FLAGS_sensor_type == "UDP") {
#ifdef __linux__  // predefined by gcc
    if (FLAGS_udp_port <= 0) {
      LOG(ERROR) << "upd_port needs to be set in UDP mode";
      return -1;
    }
#else  // __linux__
    LOG(ERROR) << "Only linux supports UDP mode";
    return -1;
#endif  // __linux__
  } else if (FLAGS_sensor_type == "OCV") {
    std::cout << "running in with opencv camera mode" << std::endl;
  } else {
    LOG(ERROR) << "sensor_type " << FLAGS_sensor_type << " not supported";
    return -1;
  }
  std::vector<std::thread> thread_pool;
  // Add IMU thread
  VLOG(1) << "thread_pool adding imu thread";
  if (FLAGS_sensor_type == "LI") {
    g_pull_imu = true;
#ifdef __linux__  // predefined by gcc
    thread_pool.push_back(std::thread(thread_pull_LI_imu, device_file_id));
    VLOG(1) << "thread_pool added thread_pull_LI_imu";
#else
    LOG(FATAL) << "Logic error";
#endif
  } else if (FLAGS_sensor_type == "XP"
             || FLAGS_sensor_type == "XP2"
             || FLAGS_sensor_type == "XP3") {
    g_pull_imu = !FLAGS_imu_from_image;
    // push the thread anyway. If pull mode fails, the thread will stop
    // and g_pull_imu will be set to false
#ifdef __linux__  // predefined by gcc
    thread_pool.push_back(std::thread(thread_pull_XP_imu, device_file_id));
    VLOG(1) << "thread_pool added thread_pull_XP_imu";
#else
    LOG(FATAL) << "logic error";
#endif
  } else if (FLAGS_sensor_type == "UDP") {
    g_pull_imu = false;
  } else if (FLAGS_sensor_type == "OCV") {
    g_pull_imu = false;
  } else {
    LOG(FATAL) << "Sensor type not supported" << FLAGS_sensor_type;
  }

  // Add imaging thread
  if (FLAGS_sensor_type == "LI"
      || FLAGS_sensor_type == "XP"
      || FLAGS_sensor_type == "XP2"
      || FLAGS_sensor_type == "XP3") {
#ifdef __linux__  // predefined by gcc
    VLOG(1) << "thread_pool adding thread_pull_img";
    thread_pool.push_back(std::thread(thread_pull_img, device_file_id, bufferinfo));
    VLOG(1) << "thread_pool added thread_pull_img";
#endif
  } else if (FLAGS_sensor_type == "UDP") {
#ifdef __linux__  // predefined by gcc
    VLOG(1) << "thread_pool adding thread_receive_udp_imgs";
    thread_pool.push_back(std::thread(thread_receive_udp_imgs));
    VLOG(1) << "thread_pool added thread_receive_udp_imgs";
#else
    LOG(FATAL) << "Logic error";
#endif
  } else if (FLAGS_sensor_type == "OCV") {
    thread_pool.push_back(std::thread(thread_stream_opencv_camera, 0));
    VLOG(1) << "thread_pool added thread_stream_opencv_camera";
  } else {
    LOG(FATAL) << "Sensor type not supported" << FLAGS_sensor_type;
  }
  thread_pool.push_back(std::thread(thread_draw_and_save_img, device_file_id));
  thread_pool.push_back(std::thread(thread_write_imu_data));
  for (auto& t : thread_pool) {
    t.join();
  }
  if (FLAGS_sensor_type == "LI"
      || FLAGS_sensor_type == "XP"
      || FLAGS_sensor_type == "XP2"
      || FLAGS_sensor_type == "XP3") {
#ifdef __linux__  // predefined by gcc
    XP_DRIVER::LI_SENSOR::stop_v4l2(&device_file_id, bufferinfo);
#endif
  }
  imgs_queue.kill();
  imu_samples_queue.kill();
  cout << "finish safe" << std::endl;
  return 0;
}
