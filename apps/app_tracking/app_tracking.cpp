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
/// \file
#include <plotting_utils.h>
#include <xp_driver_interface.h>
// UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// XP Sensor driver
#include <driver/XP_sensor_driver.h>
// XP API
#include <XP/app_api/xp_tracker.h>
#include <XP/app_api/pose_packet.h>
#include <XP/worker/actuator.h>
#include <XP/worker/actuator_impl.h>
#include <XP/helper/serial-lib.h>
// For config params
#include <XP/helper/param_internal.h>  // for NaviParam
// Parsing flags and logging
#include <gflags/gflags.h>
#include <glog/logging.h>
#ifdef HAS_ROS
#include <ros/ros.h>
#endif
#include <Eigen/Dense>
#ifdef __CYGWIN__
#include <Windows.h>  // for CPU binding
#endif
#ifdef HAS_RECOGNITION
#include <XP/util/aip/ocr.h> // NOLINT
#include <XP/util/aip/image_classify.h> // NOLINT
#include <XP/util/aip/face.h> // NOLINT
#include <json/json.h>  // NOLINT
#include <curl/curl.h> // NOLINT
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Stl used in this sample
#include <mutex>
#include <list>
#include <vector>
#include <atomic>
#include <string>
#include <algorithm>
#include <chrono>
#include <memory>  // unique_ptr
#include <thread>
#include <map>
#include <fstream>

#ifdef HAS_RECOGNITION
std::string file_content;
std::string ak = "EMPTY";
std::string sk = "EMPTY";
std::string aid = "EMPTY";
typedef cv::Point_<int> Point2i;
cv::Mat g_image_for_api;
std::thread api_thread;
bool run_flag = false;
std::mutex mt1;
#endif

using std::string;

/** \brief Whether or not to use auto gain and exposure control
 *
 * On by default
 */
DEFINE_bool(disable_auto_gain, false, "disable auto gain control");
/** \brief Path to config files
 *
 * Config files controling vio algorithm
 */
//
DEFINE_string(vio_config, "", "configuration-yaml-file");
/** \brief UDP ip address for pose service / stream
 *
 * The package sent by UDP needs better definition
 * Not setting ip but setting udp_send_to_port will start pose stream mode
 * Setting both ip and port will start pose stream mode
 */
DEFINE_string(udp_send_to_ip, "", "ip of the target machine that receives pose data via udp");
/** \brief UDP ports address for pose service / stream
 *
 * The package sent by UDP needs better definition.
 */
DEFINE_int32(udp_send_to_port, -1, "port for sending udp");
/** \brief UDP ports address to listen to
 *
 * The package sent by UDP needs better definition.
 */
DEFINE_int32(udp_listen_port, -1, "port for listening to");
/** \brief Use Harris Feature or Fast
 *
 * Use Harris is fine for PC, but too slow for ARM.
 */
DEFINE_bool(use_harris_feat, false, "whether or not use harris feat detection (v.s. fast feat)");
/** \brief Record the images, imu data, map at record_path with live tracking
 *
 * If empty, will not record anything
 */

DEFINE_string(pb_load, "",
              "Protobuf map file to load separately without entering navigation mode. "
              "To enable navigation, flag navigation_folder should be set.");

DEFINE_string(record_path, "",
              "the base folder of the recording file. Enable recording if specified");
/** \brief A special mode to only record the map (live.pb) with no images / imu data
 *
 * NOT recommended as there's no way to regenerate / refine the map once the recording session
 * is finished because missing of the raw data.
 * This flag will be ignored if record_path is NOT set.
 */
DEFINE_bool(record_map_only, false, "Enable to only record map (live.pb) w/o raw data");
/** \brief Simulate live playback of data at load_path
 *
 * If empty, will not load anything and use live sensor defined by sensor_type
 * If not empty, will load pre-recorded data and ignore live sensor input
 */
DEFINE_string(load_path, "",
              "where to load the recording file. If set, no live sensor can be used.");
/** \brief Depth file to load
 *
 * Depth is used to detect obstacles, segment objects and so on.
 */
DEFINE_string(depth_param_path, "", "where to load depth config file");
/** \brief Bag of words file to load
 *
 * Bag of words is used to relocate from getting lost as well as used for loop closure detection
 */
DEFINE_string(bow_dic_path, "", "bow dic path");
/** \brief Camera calibration file to load
 *
 * If provided, program will load from this file. Otherwise, program will try to load from sensor.
 */
DEFINE_string(calib_file, "", "camera calibration file");
/** \brief Whether or not show depth image estimated from stereo
 *
 * Not showing depth saves about 80% CPU time. Depth is not required for tracking and mapping.
 * Thus depth is not computed by default.
 */
DEFINE_bool(show_depth, false, "show depth image estimated from stereo");
/** \brief Wheter or not show both left and right image views
 *
 * If  turned on, show both left and right camera views
 */
DEFINE_bool(show_both, false, "show both left and right image views");
/** \brief Wheter or not visualize the trajectory (and depth) in 3D
 *
 * Once turned on, only the 3D visualizer will be used.
 * If HAS_OPENCV_VIZ not defined (cannot be found by CMake), this flag will be forced to false.
 */
DEFINE_bool(viz3d, false, "visualize trajectory (and depth) in 3D");
/** \brief Visualize in a simple view
 *
 * Show trajectory and left cam image in one view
 * Vio visualizer will not run in this mode
 */
DEFINE_bool(show_simple, false, "Show trajectory and left cam image in one view");
/** \brief Whether or not use IBA as the backend for VIO
 */
DEFINE_bool(iba_for_vio, false, "Use IBA as the backend for VIO for not");
/** \brief Input the navigation config file
 *
 * If empty, navigation function is turned off.
 * If provided, set up navigator / controller / actuator accordingly.
 */
DEFINE_string(navi_config, "", "Specify the navigation config file. If empty, turn off navigation");
/** \brief The UDP IP and port that the tracking app will send guide message to.
 *
 * These flags only take effect when actuator sample mode is enabled.
 */
DEFINE_string(guide_ip, "", "the ip that the UDP guide message is sent to. e.g. 127.0.0.1");
/** \brief The UDP port that the tracking app will send guide message to.
 *
 * These flags only take effect when actuator sample mode is enabled.
 */
DEFINE_int32(guide_port, -1, "the port that the UDP guide message is sent to. -1 ignore");
/** \brief The UDP port that the tracking app will receive wheel odom message.
 *
 * These flags only take effect when actuator sample mode is enabled.
 */
DEFINE_int32(recv_odom_port, -1, "the port that the UDP wheel odom message is received. -1 ignore");
/** \brief Use which type of sensor
 *
 * XP, XP2, XP3: XP, XP2, XP3 sensor
 */
DEFINE_string(sensor_type, "XP", "XP, XP2, XP3, XPIRL, XPIRL2, BoteyeOne, HTTP, ROS");
/** \brief Where does the video sensor open
 *
 */
DEFINE_string(dev_path, "", "linux file to the video cam. Empty enables auto mode");
/** \brief xp sensor device ID
 *
 */
DEFINE_string(sensor_id, "", "xp sensor ID, e.g. XPIRL3B18234005. Empty enables auto mode");
/** \brief Save every image into one file
 *
 * The existing one will be refreshed
 * This file will be read and transfered to Android
 * using another program (e.g. the python script)
 */
#ifndef __CYGWIN__
DEFINE_string(vis_img_save_path, "", "Save every visualization img"
                                     " into this single file (refresh)");
#endif

/** \brief Whether or not show displays
 *
 * Not showing X window saves computing time
 */
DEFINE_bool(no_display, false, "do not show any X windows");

/** \brief Whether or not pull imu from image data
 *
 * Not pull imu from image and use ioctl
 */
DEFINE_bool(imu_from_image, false, "Do not load imu from image");

DEFINE_string(wb_mode, "preset", "white balance mode: auto, disabled, preset");

DEFINE_string(navigation_folder, "",
              "The folder navigation related files are generated. "
              "This should usually be record_path/navigation/");

DEFINE_string(server_address, "", "Map server to connect with e.g. https://xxx:xxx");
DEFINE_string(stream_address,
              "",
              "rtmp address to stream sensor image, will auto set sensor device_id as suffix");

DEFINE_string(recv_navi_cmd_topic,
              "local_target_id",
              "Subscribe this topic to receive navigation command");
DEFINE_double(manual_control_keypress_timeout, 1.0, "Timeout key pressed after this threshold.");

// TODO(Mingyu): use a flag to turn on NcsWorker with hardcoded config for now
DEFINE_bool(use_ncs, false, "Turn on the NcsWorker with the hardcoded config for now");

/** \brief Only print the configuration of the current build of libXP and exit app_tracking
 */
DEFINE_bool(ver, false, "Print the configuration of the current build of libXP and exit");

static XP_TRACKER::VioState vio_state;

/** \brief Callback function for getting vio state
 *
 * This callback function will be registered inside the tracker.
 * \param vio_state_. pose, linear velocity, angular velocity, matched feature number
 */
void vio_state_callback(const XP_TRACKER::VioState &vio_state_) {
  vio_state = vio_state_;
}

#ifdef HAS_RECOGNITION
DEFINE_bool(face_attribute, false, "enable face attribute");
DEFINE_bool(ocr, false, "enable ocr");
DEFINE_bool(face_recognition, false, "enable face recognition");
DEFINE_bool(object_detection, false, "enable object detection");

void read_api_keys() {
  const char* homepath = getenv("HOME");
  std::string path = "/tmp/keys";
  if (homepath) {
    path = std::string(homepath);
    path += "/XP_release/keys";
  }
  std::ifstream ifs(path);
  std::string line;
  while (std::getline(ifs, line)) {
    char key[4], value[33];
    sscanf(line.c_str(), "%2s=%s", key, value);
    if (!strcmp(key, "ak")) {
      ak = std::string(value);
    } else if (!strcmp(key, "sk")) {
      sk = std::string(value);
    } else if (!strcmp(key, "id")) {
      aid = std::string(value);
    }
  }

  if (ak == "EMPTY" || sk == "EMPTY" || aid == "EMPTY") {
    LOG(ERROR) << "aid, ak or sk not set in /tmp/keys";
    exit(-1);
  }
}

void face_attribute() {
  aip::Face * aipFace = new aip::Face(aid, ak, sk);
  std::map<std::string, std::string> options;
  options["face_fields"] = "age,beauty,expression,faceshape,gender,glasses,race,qualities";
  Json::Value root = aipFace->detect(file_content, options);
  if (root["error_code"].asInt() != 0)
    std::cout << root <<std::endl;
  if (root["result"].size() == 0)
    return;
  std::string output1 =
    "age " + root["result"][0]["age"].asString() + "  " +
    "beauty " + root["result"][0]["beauty"].asString() + "  ";
  std::string output2 =
    "race " + root["result"][0]["race"].asString() + "  " +
    "gender:" + root["result"][0]["gender"].asString();
  mt1.lock();
  cv::putText(g_image_for_api, output1, cv::Point(30, 60),
    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  cv::putText(g_image_for_api, output2, cv::Point(30, 80),
    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  Point2i a(root["result"][0]["location"]["left"].asInt(),
    root["result"][0]["location"]["top"].asInt());
  Point2i b(root["result"][0]["location"]["left"].asInt()
    + root["result"][0]["location"]["width"].asInt(),
    root["result"][0]["location"]["top"].asInt() +
    root["result"][0]["location"]["height"].asInt());
  cv::rectangle(g_image_for_api, a, b, cv::Scalar(255, 255, 255), 1);
  mt1.unlock();
}

void face_recognition() {
  aip::Face * aipFace = new aip::Face(aid, ak, sk);
  Json::Value root = aipFace->identify("xteam", file_content, aip::null);
  if (root["error_code"].asInt() != 0)
    std::cout << root <<std::endl;
  if (root["result"].size() == 0)
    return;
  if (root["result"].size() > 0) {
    if (root["result"][0]["scores"][0].asFloat() < 30) {
      return;
    }
    mt1.lock();
    std::string output = root["result"][0]["uid"].asString() +
      ": " + root["result"][0]["user_info"].asString() + " Score: " +
      root["result"][0]["scores"][0].asString();
    cv::putText(g_image_for_api, output.c_str(), cv::Point(30, 450),
      cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    mt1.unlock();
  }
}

void ocr() {
  aip::Ocr * aipOcr = new aip::Ocr(aid, ak, sk);
  Json::Value root = aipOcr->general_basic(file_content, aip::null);
  if (root["error_code"].asInt() != 0)
    std::cout << root <<std::endl;
  if (root["words_result"].size() == 0)
    return;
  std::cout << (&root["words_result"])->toStyledString();
}

void thread_apis() {
  while (run_flag) {
    cv::imwrite("/tmp/1.jpg", *live::XpDriverInterface::getInstance().g_img_l_ptr);
    g_image_for_api = cv::imread("/tmp/1.jpg", CV_LOAD_IMAGE_COLOR);
    aip::get_file_content("/tmp/1.jpg", &file_content);
    std::vector<std::thread> thread_pool;
    if (FLAGS_face_attribute)
      thread_pool.push_back(std::thread(face_attribute));
    if (FLAGS_face_recognition)
      thread_pool.push_back(std::thread(face_recognition));
    if (FLAGS_ocr)
      thread_pool.push_back(std::thread(ocr));
    // object detection is postponed before AIP can provide corresponding API
    // thread_pool.push_back(std::thread(object_detection));
    for (auto& t : thread_pool) {
      t.join();
    }
    if (g_image_for_api.cols > 0)
      imshow("recognition", g_image_for_api);
    run_flag = false;
  }
}
#endif  // HAS_RECOGNITION

/// \brief udp ip and port
int g_send_guide_socket = -1;
int g_recv_odom_socket = -1;
struct sockaddr_in g_send_guide_udp_addr;
struct sockaddr_in g_recv_odom_udp_addr;
bool init_server_socket(const std::string &ip, const int port) {
  g_send_guide_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_send_guide_socket < 0) {
    LOG(ERROR) << "Could not create socket for guide command server.";
    return false;
  }
  // Prepare the sockaddr_in structure
  g_send_guide_udp_addr.sin_family = AF_INET;
  g_send_guide_udp_addr.sin_addr.s_addr = inet_addr(ip.c_str());
  g_send_guide_udp_addr.sin_port = htons(port);
  return true;
}

// TODO(hangmeng): not sure whether the config of UDP is correct
bool init_client_socket(const int port) {
  g_recv_odom_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_recv_odom_socket < 0) {
    LOG(ERROR) << "Could not create socket for wheel odom client.";
    return false;
  }
  // Prepare the sockaddr_in structure
  g_recv_odom_udp_addr.sin_family = AF_INET;
  g_recv_odom_udp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  g_recv_odom_udp_addr.sin_port = htons(port);
  if (bind(g_recv_odom_socket,
           (struct sockaddr *) &g_recv_odom_udp_addr,
           sizeof(g_recv_odom_udp_addr)) < 0) {
    LOG(ERROR) << "Fail to connect to wheel odom app";
    return false;
  }
  return true;
}

/** \brief Callback function for getting walk guide information
 *
 * The information will be sent to UDP port
 * \param guide_message The guide message produced by SDK
 */
void guide_message_callback(const XP_TRACKER::GuideMessage &guide_message) {
  if (g_send_guide_socket >= 0) {
    if (sendto(g_send_guide_socket, &guide_message, sizeof(guide_message), 0,
               (struct sockaddr *) &g_send_guide_udp_addr, sizeof(g_send_guide_udp_addr)) < 0) {
      LOG(ERROR) << "sendto() failed -> " << FLAGS_guide_ip << ":" << FLAGS_guide_port;
    }
  }
}

bool wheel_odom_callback(XP_TRACKER::WheelOdomMessage *wheel_odom_message) {
  if (g_recv_odom_socket >= 0) {
    socklen_t addr_length = sizeof(g_recv_odom_udp_addr);
    int len = recvfrom(g_recv_odom_socket, &wheel_odom_message, sizeof(wheel_odom_message), 0,
                       (struct sockaddr *) &g_recv_odom_udp_addr, &addr_length);
    if (len != sizeof(XP_TRACKER::WheelOdomMessage)) {
      LOG(ERROR) << "recvfrom() failed -> port: " << FLAGS_recv_odom_port;
      return false;
    } else {
      return true;
    }
  }
  return false;
}

XP::MouseData g_mouse_data;
void mouse_data_callback(int event, int x, int y, int flags, void *mouse_data_ptr) {
  XP::MouseData *mouse_data = reinterpret_cast<XP::MouseData *>(mouse_data_ptr);
  if (event == cv::EVENT_LBUTTONDOWN) {
    mouse_data->pixel_x = x;
    mouse_data->pixel_y = y;
    mouse_data->mouse_pressed = true;
  }
}

/** \brief  Main function
 * \param  argc An integer argument count of the command line arguments
 * \param  argv An argument vector of the command line arguments
 * \return an integer 0 upon exit success
 */
int main(int argc, char **argv) {
#ifdef __CYGWIN__
  HANDLE process_handle = GetCurrentProcess();
  DWORD_PTR thread_affinity_mask = 0xFF;  // the first 8 cores
  if (!SetProcessAffinityMask(process_handle, thread_affinity_mask)) {
    LOG(ERROR) << "!SetThreadAffinityMask " << thread_affinity_mask;
  }
#endif
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::SetUsageMessage("Baidu Boteye SDK tracking / navigation tool");
  FLAGS_colorlogtostderr = 1;
#ifdef HAS_ROS
  // Setup ROS node
  int ros_init_flag = 0;
  ros::init(ros_init_flag, nullptr, "app_tracking");
  if (!ros::master::check()) {
    LOG(ERROR) << "ROS master has not started yet. "
                  "Please run roscore and restart app_tracking.";
    return -1;
  }
  ros::NodeHandle nh("boteye");
#endif
#ifndef HAS_OPENCV_VIZ
  if (FLAGS_viz3d) {
    FLAGS_viz3d = false;  // Opencv_viz is NOT available
    LOG(ERROR) << "Cannot find opencv_viz. viz3d is invalid";
  }
#endif  // HAS_OPENCV_VIZ

  if (FLAGS_ver) {
    XP_TRACKER::print_XP_configuration();
    return 0;
  }

  // Initialize XP sensor (or load from files)
  if (!FLAGS_load_path.empty()) {
    if (!live::XpDriverInterface::getInstance().init_data_loader(FLAGS_load_path)) {
      LOG(ERROR) << "!live::XpDriverInterface::getInstance().init_data_loader()";
      // TODO(mingyu): Check the image resolution
      return -1;
    }
  } else {
    // [NOTE] This is the place to swap in a custom hardware if NOT using XP sensor
    if (FLAGS_sensor_type == "HTTP") {
      if (!live::XpDriverInterface::getInstance().init_http_sensor()) {
        LOG(ERROR) << "!live::init_http_sensor()";
        return -1;
      }
    } else if (FLAGS_sensor_type == "ROS") {
      if (!live::XpDriverInterface::getInstance().init_ros_subscriber()) {
        LOG(ERROR) << "!live::init_ros_subscriber()";
        return -1;
      }
    } else if (!live::XpDriverInterface::getInstance().init_XP_sensor(FLAGS_sensor_type,
                                                                      !FLAGS_disable_auto_gain,
                                                                      FLAGS_imu_from_image,
                                                                      FLAGS_dev_path,
                                                                      FLAGS_sensor_id,
                                                                      FLAGS_wb_mode)) {
      LOG(ERROR) << "!live::init_XP_sensor()";
      return -1;
    }

    // We have to initialize NcsWorker before registering data callback in order to pass
    // images to NcsWorker.
    // FLAGS_use_ncs will be overwritten to false if NcsWorker cannot be initialized correctly.
    if (FLAGS_use_ncs) {
      // TODO(mingyu): hardcoded the graph parameters for now
      float network_means[3] = {127.5f, 127.5f, 127.5f};
      float network_scales[3] = {0.007843, 0.007843, 0.007843};
      const std::string graph_filename =
          "/home/mchen/Development/tracking/models/SSD_MobileNet/graph";
      FLAGS_use_ncs =
          XP_TRACKER::init_ncs_worker("DetectionOutput",
                                      graph_filename,
                                      300, 300,
                                      network_means,
                                      network_scales);
    }
  }

  // If FLAGS_calib_file is not provided, try to load from
  // 1) the calibration from the sensor (live mode)
  // 2) the calibration from the load path (playback mode)
  XP::DuoCalibParam calib_param;
  if (FLAGS_calib_file.empty() && FLAGS_load_path.empty()) {
    if (!live::XpDriverInterface::getInstance().auto_calib_load(&calib_param)) {
      LOG(ERROR) << "Cannot load calib from the sensor.  Must provide a calib_file!";
      return -1;
    }
  } else if (FLAGS_calib_file.empty() && !FLAGS_load_path.empty()) {
    if (!live::XpDriverInterface::getInstance().load_calib_from_folder(FLAGS_load_path,
                                                                       &calib_param)) {
      return -1;
    }
  } else if (!live::XpDriverInterface::getInstance().load_calib_from_file(FLAGS_calib_file,
                                                                          &calib_param)) {
    return -1;
  }

  // Set the properties of cameras
  const int img_h = calib_param.Camera.img_size.height;
  const int img_w = calib_param.Camera.img_size.width;
  const int display_h = std::max(img_h, 480);  // We set the minimal display size
  const int display_w = std::max(img_w, 640);
#ifdef __linux__
  const char *env_display_p = std::getenv("DISPLAY");
  if (env_display_p == nullptr) {
    std::cout << "You are running headless OS. No X windows will be shown." << std::endl;
    FLAGS_no_display = true;
  }
#endif

  if (FLAGS_vio_config.empty()) {
    const char *env_p = std::getenv("MASTER_DIR");
    if (env_p == nullptr) {
      LOG(ERROR) << "You need to set MASTER_DIR. Did you forget"
                    " to run 'source config_environment.sh'?";
      return -1;
    }
    FLAGS_vio_config = std::string(env_p);
    FLAGS_vio_config += "/vio/config/config_default.yaml";
    LOG(INFO) << "Default config " << FLAGS_vio_config;
  }
  if (FLAGS_depth_param_path.empty()) {
    const char *env_p = std::getenv("MASTER_DIR");
    if (env_p == nullptr) {
      LOG(ERROR) << "You need to set MASTER_DIR. Did you forget"
                    " to run 'source config_environment.sh'?";
      return -1;
    }
    FLAGS_depth_param_path =
        std::string(env_p) + "/XP/config/depth_param.yaml";
    LOG(INFO) << "Default config " << FLAGS_depth_param_path;
  }
  if (FLAGS_bow_dic_path.empty()) {
    const char *env_p = std::getenv("HOME");
    if (env_p == nullptr) {
      LOG(ERROR) << "You need to set bow_dic_path, which usually is "
                    "~/XP_release/3rdparty_lib_lean/BOW.proto";
      return -1;
    }
    FLAGS_bow_dic_path =
        std::string(env_p) + "/XP_release/3rdparty_lib_lean/BOW.proto";
    LOG(INFO) << "bow_dic_path is unset. Try " << FLAGS_bow_dic_path;
  }

  // Whether or not ESC is pressed
  bool ESC_pressed = false;
  if (!XP_TRACKER::init_tracker(FLAGS_vio_config,
                                FLAGS_bow_dic_path,
                                FLAGS_depth_param_path,
                                calib_param,
                                !FLAGS_use_harris_feat,
                                FLAGS_iba_for_vio)) {
    LOG(ERROR) << "Init tracker failed";
    return -1;
  }
  XP_TRACKER::set_data_rate_callback([](float *img_rate,
                                        float *imu_rate) {
    live::XpDriverInterface::getInstance().get_data_rate(img_rate, imu_rate);
  });

  if (FLAGS_udp_send_to_port > 0) {
    if (FLAGS_udp_send_to_ip.empty()) {
      if (!XP_TRACKER::init_udp_service(FLAGS_udp_send_to_port)) {
        LOG(ERROR) << "Init udp service failed";
      }
    } else {
      if (!XP_TRACKER::init_udp_stream(FLAGS_udp_send_to_ip, FLAGS_udp_send_to_port)) {
        LOG(ERROR) << "Init udp stream failed";
      }
    }
  }
  if (FLAGS_udp_listen_port > 0) {
    if (!XP_TRACKER::init_udp_listen(FLAGS_udp_listen_port)) {
      LOG(ERROR) << "init_udp_listen failed";
    }
  }

  if ((!FLAGS_pb_load.empty()) &&
      (FLAGS_navigation_folder.empty())) {
    LOG(INFO) << "Mode: loading map pb without navigation.";
    if (!XP_TRACKER::load_map(FLAGS_pb_load)) {
      LOG(ERROR) << "Loading map pb failed";
    } else {
      LOG(INFO) << "Loaded " << FLAGS_pb_load;
    }
  }

  // [Optional] Configure map loading and navigation
  bool is_navigation_set = false;
  if (!FLAGS_navigation_folder.empty()) {
    string pb_load = FLAGS_navigation_folder + "/navi.pb";
    if (!XP_TRACKER::load_map(pb_load)) {
      LOG(ERROR) << "Loading map failed";
    } else {
      LOG(INFO) << "Loaded " << FLAGS_pb_load;
    }
    if (!FLAGS_server_address.empty()) {
      std::string device_id = "unknown";
      live::XpDriverInterface::getInstance().get_sensor_deviceid(&device_id);
      if (!XP_TRACKER::init_robot_client(FLAGS_server_address,
                                         FLAGS_stream_address,
                                         device_id,
                                         static_cast<unsigned int>(img_w),
                                         static_cast<unsigned int>(img_h))) {
        LOG(ERROR) << "init_robot_client failed";
        return -1;
      }
    }

    if (!FLAGS_navi_config.empty()) {
      XP::NaviParam navi_param;
      CHECK(navi_param.LoadFromYaml(FLAGS_navi_config)) << "Fail to load " << FLAGS_navi_config;
      LOG(INFO) << FLAGS_navi_config << " is loaded ";

      if (navi_param.Navigation.type == "navi") {
        std::shared_ptr<XP::Actuator> actuator_ptr;
        if (navi_param.Actuator.type == "sample") {
          if (!FLAGS_guide_ip.empty() && FLAGS_guide_port >= 0) {
            if (!init_server_socket(FLAGS_guide_ip, FLAGS_guide_port)) {
              return -1;
            }
          }
          if (FLAGS_recv_odom_port >= 0) {
            if (!init_client_socket(FLAGS_recv_odom_port)) {
              return -1;
            }
          }
          actuator_ptr.reset(new XP::SampleActuator(navi_param.Actuator,
                                                    navi_param.Lidar,
                                                    guide_message_callback,
                                                    wheel_odom_callback));
        } else if (navi_param.Actuator.type == "reeman" ||
            navi_param.Actuator.type == "dilili") {
          actuator_ptr.reset(new XP::KincoActuator(navi_param.Actuator, navi_param.Lidar));
        } else if (navi_param.Actuator.type == "eai") {
          actuator_ptr.reset(new XP::EaiActuator(navi_param.Actuator, navi_param.Lidar));
        } else if (navi_param.Actuator.type == "gyroor") {
          actuator_ptr.reset(new XP::GyroorActuator(navi_param.Actuator, navi_param.Lidar));
#ifdef HAS_ROS
        } else if (navi_param.Actuator.type == "turtlebot3") {
          actuator_ptr.reset(new XP::RosActuator(navi_param.Actuator, navi_param.Lidar));
        } else if (navi_param.Actuator.type == "xiaodu_ros") {
          actuator_ptr.reset(new XP::RosXiaoduActuator(navi_param.Actuator, navi_param.Lidar));
#endif
        } else {
          // [NOTE] Insert the user-customized actuator here
          LOG(ERROR) << "Not supported actuator type in current build of XP: "
                     << navi_param.Actuator.type;
          return -1;
        }
        is_navigation_set =
            XP_TRACKER::set_navigator(actuator_ptr, FLAGS_navigation_folder, navi_param);
      } else {
        LOG(ERROR) << "Not supported navigation type: " << navi_param.Navigation.type;
        return -1;
      }
      if (navi_param.Navigation.use_trajectory_file) {
        // XP_TRACKER::set_navigator sets the planned_trajectory file through
        // XP_TRACKER::set_navigator_trajectory sets the planned_trajectory file for navigator.
        if (!XP_TRACKER::set_navigator_trajectory(FLAGS_navigation_folder)) {
          LOG(ERROR) << "Failed to set navigation trajectory from file, "
                     << "please set it through GUI.";
        }
      }
      if (!is_navigation_set) {
        LOG(ERROR) << "Set navigation type: " << navi_param.Navigation.type << " failed";
      }
    } else {
      LOG(ERROR) << "You must specific a navigation_param.yaml file as navi_config";
      return -1;
    }
  }

#ifdef HAS_RECOGNITION
  if (FLAGS_face_recognition || FLAGS_face_attribute || FLAGS_ocr) {
    read_api_keys();
  }
#endif
  // set live recording path if not empty
  // [NOTE] Calling multiple times of run_tracker_MT will overwrite and only record the
  //        latest session.
  // [NOTE] rec_path may be automatically appended with date + time if FLAGS_record_path
  //        already exists.
  std::string rec_path = "";
  if (!FLAGS_record_path.empty()) {
    rec_path = XP_TRACKER::set_record_path(FLAGS_record_path,
                                           FLAGS_calib_file,
                                           FLAGS_record_map_only);
  }

  cv::Mat top_down_and_camera_canvas;
  cv::Mat camera_view_canvas;
  cv::Mat depth_view_canvas;
  cv::Mat navi_canvas;
  cv::Mat top_down_view_canvas;
  cv::Mat inference_canvas;
  // init Viz3d in headless OS will crash the system
  float viz_cam_height = 1;  // in meter
#ifdef HAS_OPENCV_VIZ
  std::unique_ptr<PoseDrawer3D> pose_viewer_3d_ptr;
#endif

#ifdef HAS_ROS
  ros::Subscriber navi_cmd_sub =
    nh.subscribe(FLAGS_recv_navi_cmd_topic, 1, XP_TRACKER::ros_navi_cmd_callback);
#endif
  if (!FLAGS_no_display) {
    // Prepare viz3D or 2D draw-to canvas
    // show vio trajectory and mapper world
    // Displaying multiple windows with imshow is not efficient
    // It is desired to concatenate all images into a single one and display only the whole picture
#ifdef HAS_OPENCV_VIZ
    // viz_cam_height will increase as the scene grows bigger
    if (FLAGS_viz3d) {
      float K_left_array[9];
      XP_TRACKER::get_cam_K(0, K_left_array);
      pose_viewer_3d_ptr.reset(new PoseDrawer3D(viz_cam_height,
                                                cv::Matx33f(K_left_array)));
    } else {
#else
    {
#endif
      // Initialize top_down_and_camera_canvas
      if (FLAGS_show_simple) {
        top_down_and_camera_canvas.create(display_h, display_w, CV_8UC3);
        // In simple view mode, trajectory view is overlaid on top of the left camera view
        top_down_view_canvas = top_down_and_camera_canvas;
        camera_view_canvas = top_down_and_camera_canvas;
        XP_TRACKER::set_top_view_canvas(&top_down_view_canvas, FLAGS_show_simple);
      } else {
        // Use tracker drawer
        // do not show trajectory history
        if (FLAGS_show_both) {
          top_down_and_camera_canvas.create(display_h, display_h + display_w * 2, CV_8UC3);
        } else {
          top_down_and_camera_canvas.create(display_h, display_h + display_w, CV_8UC3);
        }
        top_down_view_canvas =
            top_down_and_camera_canvas(cv::Rect(0, 0, display_h, display_h));
        // Trajectory view is an independent view
        XP_TRACKER::set_top_view_canvas(&top_down_view_canvas, FLAGS_show_simple);

        // set to black
        top_down_and_camera_canvas.setTo(cv::Vec3b(0, 0, 0));
        cv::putText(top_down_and_camera_canvas, "Waiting for initialization",
                    cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 1);
        // Set camera_view_canvas as a sub-matrix of top_down_and_camera_canvas
        if (FLAGS_show_both) {
          camera_view_canvas =
              top_down_and_camera_canvas(cv::Rect(top_down_view_canvas.cols, 0,
                                                  display_w * 2, display_h));
          XP_TRACKER::set_camera_view_canvas(&camera_view_canvas, 2);
        } else {
          camera_view_canvas =
              top_down_and_camera_canvas(cv::Rect(top_down_view_canvas.cols, 0,
                                                  display_w, display_h));
          XP_TRACKER::set_camera_view_canvas(&camera_view_canvas, 1);
        }
        cv::namedWindow("Top View & Left Camera");
        cv::moveWindow("Top View & Left Camera", 0, 0);
      }  // FLAGS_show_simple
      cv::imshow("Top View & Left Camera", top_down_and_camera_canvas);
    }  // FLAGS_viz3d
    // Set depth canvas to start depth computation in tracker
    if (FLAGS_show_depth) {
      // depth is computed on half resolution
      // Set depth view canvas even if viz3d is used
      depth_view_canvas.create(img_h, img_w, CV_8UC3);
      XP_TRACKER::set_depth_view_canvas(&depth_view_canvas);
      if (!FLAGS_viz3d) {
        cv::namedWindow("Depth");
        cv::moveWindow("Depth", 500, 500);
      }
    }
    if (is_navigation_set) {
      navi_canvas.create(display_h, display_h + display_w, CV_8UC3);
      XP_TRACKER::set_navi_canvas(&navi_canvas);
      if (!FLAGS_viz3d) {
        cv::namedWindow("Navigator");
        cv::moveWindow("Navigator", 0, 600);
        cv::setMouseCallback("Navigator", mouse_data_callback, &g_mouse_data);
      }
    }
    if (FLAGS_use_ncs) {
      inference_canvas.create(img_h, img_w, CV_8UC3);
      XP_TRACKER::set_ncs_canvas(&inference_canvas);
    }
  }  // !FLAGS_no_display
  XP_TRACKER::set_vio_state_callback(std::bind(&vio_state_callback, std::placeholders::_1));

  live::XpDriverInterface::getInstance().run();  // Start spinning the XP sensor

  if (!XP_TRACKER::run_tracker_MT()) {
    LOG(ERROR) << "run_tracker_MT failed";
    return -1;
  }
  // cache for depth result
  cv::Mat_<cv::Vec3f> depth_result_img;
  if (!FLAGS_no_display) {
    char key_pressed = 0;
    // usleep(50000);
    int draw_counter = 0;
    std::chrono::steady_clock::time_point manual_action_tp = std::chrono::steady_clock::now();
    while (!ESC_pressed) {
#ifdef HAS_ROS
      ros::spinOnce();
#endif
      // A simple illustration on how use depth view to detect obstacle
      if (FLAGS_show_depth) {
        constexpr float alert_depth = 1;
        if (XP_TRACKER::get_depth_img(&depth_result_img)) {
          // if the avg depth of the middle part of depth_result_img
          // is smaller than alert_depth, print a warning
          int pixel_counter = 0;
          float depth_sum = 0;
          for (int x = depth_result_img.cols / 3;
               x < depth_result_img.cols * 2 / 3;
               ++x) {
            for (int y = depth_result_img.rows / 3;
                 y < depth_result_img.rows * 2 / 3;
                 ++y) {
              if (depth_result_img(y, x)[2] > 1e-3) {
                depth_sum += depth_result_img(y, x)[2];
                ++pixel_counter;
              } else {
                // otherwise the depth of this pixel could not be computed
              }
            }
          }
          if (depth_sum / pixel_counter < alert_depth) {
            std::cout << "Warning: Obstacle detected. Distance "
                      << depth_sum / pixel_counter << std::endl;
          }
        }
      }
      if (FLAGS_viz3d) {
#ifdef HAS_OPENCV_VIZ
        if (pose_viewer_3d_ptr != nullptr) {
          ESC_pressed = (pose_viewer_3d_ptr->key_pressed() == 27);
          static float W_T_D_mapper[16];
          if (XP_TRACKER::get_mapper_latest_3d_pose(W_T_D_mapper)) {
            cv::Affine3f cam_pose(W_T_D_mapper);
            // re-draw traj every 10 frames (~0.5 sec)
            cv::Mat rig_xyz_mat;
            // re-draw traj every 10 frames (~0.5 sec)
            if (draw_counter % 10 == 0) {
              XP_TRACKER::get_mapper_key_rigs_pos(&rig_xyz_mat);
            } else {
              // otherwise pass an empty mat into the function
            }
            pose_viewer_3d_ptr->viz3d_once(cam_pose,
                                           *live::XpDriverInterface::getInstance().g_img_l_ptr,
                                           rig_xyz_mat,
                                           depth_result_img);
          }
        }
#endif
      } else {
        // top_down_view_canvas is drawn within tracker
        if (!XP_TRACKER::draw_once()) {
          // very likely tracker stops running
          std::cout << "!XP_TRACKER::draw_once()" << std::endl;
          // sleep for 1 sec so all threads can stop properly
          sleep(1);
          ESC_pressed = true;
        }
        cv::imshow("Top View & Left Camera", top_down_and_camera_canvas);
#ifndef __CYGWIN__
        if (!FLAGS_vis_img_save_path.empty()) {
          cv::imwrite(FLAGS_vis_img_save_path, top_down_and_camera_canvas);
        }
#endif
        if (FLAGS_show_depth) {
          cv::imshow("Depth", depth_view_canvas);
        }
        if (is_navigation_set) {
          cv::imshow("Navigator", navi_canvas);
        }
        if (FLAGS_use_ncs) {
          cv::imshow("Inference", inference_canvas);
        }
        key_pressed = 0x00;
#ifdef __ARM_NEON__
        // Wait for longer time on ARM
        key_pressed = cv::waitKey(50);
#else
        // Wait for only a short time
        try {
          key_pressed = cv::waitKey(5);
        } catch (...) {}
#endif

        if (g_mouse_data.mouse_pressed) {
          XP_TRACKER::set_navigator_mouse_data(g_mouse_data);
          g_mouse_data.mouse_pressed = false;  // reset
        }
        if (key_pressed == 27) {
          // viz3d may also set ESC_pressed
          ESC_pressed = true;
        } else if (key_pressed == 'A' || key_pressed == 'a' ||
            key_pressed == 'B' || key_pressed == 'b' ||
            key_pressed == 'C' || key_pressed == 'c' ||
            key_pressed == 'D' || key_pressed == 'd' ||
            key_pressed == 'E' || key_pressed == 'e') {
          if (FLAGS_navi_config.empty()) {
            // not in navigation mode
            // mark the latest KF
            // if there is alreayd a KF with this tag, force the new KF to share the same position
            // with the previous one. This is useful to correct path drift
            if (!XP_TRACKER::send_command_to_mapper("AddTagToLastKeyFrame", key_pressed)) {
              LOG(ERROR) << "send_command_to_mapper AddTagToLastKeyFrame failed";
            }
          }
        } else if (key_pressed == 'S' || key_pressed == 's') {
          XP_TRACKER::set_static(true);
        } else if (key_pressed == 'W' || key_pressed == 'w') {
          XP_TRACKER::set_static(false);
#ifdef HAS_RECOGNITION
        } else if (key_pressed == 'r') {
          run_flag = !run_flag;
          if (run_flag) {
            std::thread(thread_apis).detach();
          }
#endif
        } else if (key_pressed == 10) {
          // CR is pressed
          XP_TRACKER::finish_set_loop_targets();
        } else if (key_pressed == 'P' || key_pressed == 'p') {
          // {P}rint the current actuator 2D pose as a local target pose and record it into a file (
          // "/tmp/target_id_pose.txt").  Require to use navigator.
          if (!XP_TRACKER::print_current_local_target_2d_pose()) {
            LOG(ERROR) << "print_current_local_target_2d_pose NOT working";
          }
        } else if (key_pressed == 'M' || key_pressed == 'm') {
          // {M}ark the current mapper pose in the map.  The marked waypoints (poses) will be
          // adjusted by loop closure accordingly if not pre-built.
          if (!XP_TRACKER::mark_current_waypoint()) {
            LOG(ERROR) << "mark_current_waypoint NOT working";
          }
          // TODO(meng): add fast speed level logic
        } else if (key_pressed == 'I' || key_pressed == 'i') {
          // up is pressed
          manual_action_tp = std::chrono::steady_clock::now();
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::MANUAL);
          XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::FORWARD_NORMAL,
                                                  XP_TRACKER::AngularVelocityLevel::IDLE);
        } else if (key_pressed == 'K' || key_pressed == 'k') {
          // down is pressed
          manual_action_tp = std::chrono::steady_clock::now();
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::MANUAL);
          XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::BACKWARD_NORMAL,
                                                  XP_TRACKER::AngularVelocityLevel::IDLE);
        } else if (key_pressed == 'J' || key_pressed == 'j') {
          // left is pressed
          manual_action_tp = std::chrono::steady_clock::now();
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::MANUAL);
          XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::IDLE,
                                                  XP_TRACKER::AngularVelocityLevel::LEFT_NORMAL);
        } else if (key_pressed == 'L' || key_pressed == 'l') {
          // right is pressed
          manual_action_tp = std::chrono::steady_clock::now();
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::MANUAL);
          XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::IDLE,
                                                  XP_TRACKER::AngularVelocityLevel::RIGHT_NORMAL);
        } else if (key_pressed == ' ' || key_pressed == 32) {
          // idle is pressed
          std::cout << "exit manual control mode" << std::endl;
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::MANUAL);
          XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::IDLE,
                                                  XP_TRACKER::AngularVelocityLevel::IDLE);
        } else if (key_pressed == 'N' || key_pressed == 'n') {  // "N" stands for navigation
          XP_TRACKER::set_navigator_motion_mode(XP_TRACKER::MotionMode::CONTROL);
        } else if (key_pressed == static_cast<char>(-1)) {
          // nothing is pressed
          std::chrono::steady_clock::time_point curr_tp = std::chrono::steady_clock::now();
          float no_key_input_sec =
            std::chrono::duration_cast<std::chrono::seconds>(curr_tp - manual_action_tp).count();
          if (no_key_input_sec > FLAGS_manual_control_keypress_timeout) {
            manual_action_tp = std::chrono::steady_clock::now();
            // only do this when robot_client is disabled
            if (FLAGS_server_address.empty()) {
              XP_TRACKER::set_navigator_manual_action(XP_TRACKER::LinearVelocityLevel::IDLE,
                                                      XP_TRACKER::AngularVelocityLevel::IDLE);
            }
          }
        } else {
          LOG(ERROR) << "Key " << static_cast<int32_t>(key_pressed)
                     << " is pressed but unknown";
        }
      }
      ++draw_counter;
    }
    cv::destroyAllWindows();
  } else {
    // FLAGS_no_display is set (HEADLESS mode)
    while (true) {
      if (!XP_TRACKER::is_duo_vio_tracker_running()) {
        std::cout << "duo_vio_tracker stops running." << std::endl;
        break;
      }
#ifdef HAS_ROS
      ros::spinOnce();
#endif
      usleep(100000);  // 10 Hz
    }
  }

  // Save the final mapper trajectory before exit
  if (!FLAGS_no_display) {
    if (rec_path.empty()) {
      cv::imwrite("/tmp/final_traj.png", top_down_view_canvas);
    } else {
      cv::imwrite(rec_path + "/final_traj.png", top_down_view_canvas);
    }
  }

  if (!rec_path.empty()) {
    // We always save the map to live.pb at rec_path
    string pb_save = rec_path + "/live.pb";
    if (!XP_TRACKER::save_map(pb_save)) {
      LOG(ERROR) << "Save map failed " << pb_save;
    }
  }

  XP_TRACKER::stop_tracker("app_tracking main");
  live::XpDriverInterface::getInstance().stop();  // Stop spinning XP sensor or http stream
  // Linux crashes at the end if release is not explicitly called
  live::XpDriverInterface::getInstance().g_img_l_ptr->release();
  return 0;
}  // NOLINT
