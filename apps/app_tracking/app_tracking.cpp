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
/// \file
#include <plotting_utils.h>
// UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
// XP API
#include <XP/app_api/xp_tracker.h>
#include <XP/app_api/pose_packet.h>
// Parsing flags and logging
#include <gflags/gflags.h>
#include <glog/logging.h>
// Stl used in this sample
#include <mutex>
#include <list>
#include <vector>
#include <atomic>
#include <string>
#include <algorithm>
#include <chrono>
#include <fstream>  // NOLINT
#include <memory>  // unique_ptr
#ifdef __CYGWIN__
#include <Windows.h>  // for CPU binding
#endif
#ifdef HAS_RECOGNITION
#include <XP/util/aip/ocr.h>
#include <XP/util/aip/image_classify.h>
#include <XP/util/aip/face.h>
#include <json/json.h>
#include <curl/curl.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
// #include <XP/util/i18nText.h>
#include <thread>

std::string file_content;
std::string ak = "EMPTY";
std::string sk = "EMPTY";
std::string aid = "EMPTY";
// i18nText i18n;
typedef cv::Point_<int> Point2i;
cv::Mat image_for_api(480, 640, CV_8UC1);
std::thread api_thread;
bool run_flag = false;
std::mutex mt1;
#endif
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
/** \brief Map file for loading
 *
 * pb stands for protobuf file
 */
DEFINE_string(pb_load, "", "Protobuf map file to load");
/** \brief Map file to save
 *
 * pb stands for protobuf file
 */
DEFINE_string(pb_save, "", "Protobuf map file to save at the end");
/** \brief Record the latest tracking live at record_path
 *
 * If empty, will not record anything
 */
DEFINE_string(record_path, "",
              "the base folder of the recording file. Enable recording if specified");
/** \brief Bag of words file to load
 *
 * Bag of words is used to relocate from getting lost as well as used for loop closure detection
 */
DEFINE_string(bow_dic_path, "", "bow dic path");
/** \brief Camera calibration file to load
 *
 * Must provide it.
 */
DEFINE_string(cam_calib_path, "", "camera calibration file");
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
/** \brief Use which type of path follower
 *
 * If not specified (empty), path follower is disabled.
 * Currently we support "robot" (EAI dashgo) and "walk".
 */
DEFINE_string(path_follower, "", "specify path follower type: robot, walk");
/** \brief The UDP IP and port that the tracking app will send guide message to.
 *
 * These flags only take effect when path_follower walk is enabled.
 */
DEFINE_string(guide_ip, "", "the ip that the UDP guide message is sent to.e.g. 127.0.0.1");
/** \brief The UDP port that the tracking app will send guide message to.
 *
 * These flags only take effect when path_follower walk is enabled.
 */
DEFINE_int32(guide_port, -1, "the port that the UDP guide message is sent to. -1 ignore");
/** \brief Use which type of sensor
 *
 * LI: LI sensor
 * XP: XP sensor
 * OCV: use openCV to run XP sensor (experimental)
 */
DEFINE_string(sensor_type,
#ifdef __linux__
  "XP",  // XP camera module is the default sensor in Linux
#else
  "OCV",  // use opencv driver (experimental) for non-linux
#endif
  "LI, XP, XP2, OCV");
/** \brief Where does the video sensor open
 *
 */
DEFINE_string(sensor_dev_path, "", "linux file to the video cam. Empty enables auto mode");
/** \brief Where does the imu open
 *
 */
DEFINE_string(imu_dev_path, "", "which imu dev to open. e.g. "
                                "/dev/ttyUSB1. Does not apply to LI sensor. "
                                "If this is unset for XP sensor, the IMU data will be read "
                                " from imgs");
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

/** \brief Callback function for getting raw images
 *
 * This callback function will be registered inside the tracker.
 * \param img_l left image
 * \param img_r right image
 * \param timestamp the timetamp (in seconds) of the images
 */
static std::shared_ptr<cv::Mat_<uchar>> g_img_l_ptr;
void raw_sensor_img_callback(const cv::Mat_<uchar>& img_l,
                             const cv::Mat_<uchar>& img_r,
                             float timestamp) {
  if (g_img_l_ptr == nullptr) return;
  if (g_img_l_ptr->rows == 0) {
    g_img_l_ptr->create(img_l.size());
  }
  img_l.copyTo(*g_img_l_ptr);
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
  cv::putText(image_for_api, output1, cv::Point(30, 60),
    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  cv::putText(image_for_api, output2, cv::Point(30, 80),
    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  Point2i a(root["result"][0]["location"]["left"].asInt(),
    root["result"][0]["location"]["top"].asInt());
  Point2i b(root["result"][0]["location"]["left"].asInt()
    + root["result"][0]["location"]["width"].asInt(),
    root["result"][0]["location"]["top"].asInt() +
    root["result"][0]["location"]["height"].asInt());
  cv::rectangle(image_for_api, a, b, cv::Scalar(255, 255, 255), 1);
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
    cv::putText(image_for_api, output.c_str(), cv::Point(30, 450),
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
      cv::imwrite("/tmp/1.jpg", *g_img_l_ptr);
      image_for_api = cv::imread("/tmp/1.jpg", CV_8UC1);
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
      if (image_for_api.cols > 0)
        imshow("recognition", image_for_api);
      run_flag = false;
  }
}

#endif
/// \brief udp ip and port
int g_udp_socket = -1;
struct sockaddr_in g_udp_addr;
/** \brief Callback function for getting walk guide information
 *
 * The information will be sent to UDP port
 * \param guide_message The guide message produced by SDK
 */
void path_follower_walk_callback(const XP_TRACKER::GuideMessage& guide_message) {
  if (FLAGS_guide_ip.empty() || FLAGS_guide_port < 0) {
    // If path follower is enabled but no udp ip and port is set,
    // print info here and return
    std::cout << "guide msg: " << guide_message.status << std::endl;
    return;
  }
  if (g_udp_socket < 0) {
    // init
    g_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_socket < 0) {
      LOG(ERROR) << "Could not create socket for PathFollwerWalk";
      return;
    }
    // Prepare the sockaddr_in structure
    g_udp_addr.sin_family = AF_INET;
    g_udp_addr.sin_addr.s_addr = inet_addr(FLAGS_guide_ip.c_str());
    g_udp_addr.sin_port = htons(FLAGS_guide_port);
  }
  if (sendto(g_udp_socket, &guide_message, sizeof(guide_message), 0,
             (struct sockaddr *)&g_udp_addr, sizeof(g_udp_addr)) < 0) {
    LOG(ERROR) << "sendto() failed -> " << FLAGS_guide_ip << ":" << FLAGS_guide_port;
    return;
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
  FLAGS_colorlogtostderr = 1;
#ifndef HAS_OPENCV_VIZ
  if (FLAGS_viz3d) {
    FLAGS_viz3d = false;  // Opencv_viz is NOT available
    LOG(ERROR) << "Cannot find opencv_viz. viz3d is invalid";
  }
#endif  // HAS_OPENCV_VIZ
  if (FLAGS_sensor_type == "LI") {
    if (!XP_TRACKER::init_sensor_LI(FLAGS_sensor_dev_path,
                                    !FLAGS_disable_auto_gain)) {
      LOG(ERROR) << "!XP_TRACKER::init_sensor_LI()";
      return -1;
    }
  } else if (FLAGS_sensor_type == "XP") {
    if (!XP_TRACKER::init_sensor_XP(FLAGS_sensor_dev_path,
                                    FLAGS_imu_dev_path,
                                    !FLAGS_disable_auto_gain,
                                    FLAGS_imu_from_image)) {
      LOG(ERROR) << "!XP_TRACKER::init_sensor_XP()";
      return -1;
    }
  } else if (FLAGS_sensor_type == "XP2") {
    if (!XP_TRACKER::init_sensor_XP2(FLAGS_sensor_dev_path,
                                    FLAGS_imu_dev_path,
                                    !FLAGS_disable_auto_gain,
                                    FLAGS_imu_from_image)) {
      LOG(ERROR) << "!XP_TRACKER::init_sensor_XP2()";
      return -1;
    }
  } else if (FLAGS_sensor_type == "OCV") {
    if (!XP_TRACKER::init_sensor_OCV(!FLAGS_disable_auto_gain)) {
      LOG(ERROR) << "!XP_TRACKER::init_sensor_OCV()";
      return -1;
    }
  } else {
    LOG(ERROR) << "sensor_type " << FLAGS_sensor_type << " not supported";
    return -1;
  }
  if (FLAGS_vio_config.empty()) {
    const char* env_p = std::getenv("MASTER_DIR");
    if (env_p == nullptr) {
      LOG(ERROR) << "You need to set MASTER_DIR. Did you forget"
                    " to run 'source config_environment.sh'?";
      return -1;
    }
    FLAGS_vio_config = std::string(env_p);
    FLAGS_vio_config += "/vio/config/config_default.yaml";
    LOG(INFO) << "Default config " << FLAGS_vio_config;
  }
  if (FLAGS_bow_dic_path.empty()) {
    const char* env_p = std::getenv("HOME");
    if (env_p == nullptr) {
      LOG(ERROR) << "You need to set bow_dic_path, which usually is "
                    "~/XP_release/3rdparty_lib_lean/BOW.proto";
      return -1;
    }
    FLAGS_bow_dic_path =
        std::string(env_p) + "/XP_release/3rdparty_lib_lean/BOW.proto";
    LOG(INFO) << "bow_dic_path is unset. Try " << FLAGS_bow_dic_path;
  }
  /// Wether or not ESC is pressed
  bool ESC_pressed = false;
  if (!XP_TRACKER::init_tracker(FLAGS_vio_config,
                                FLAGS_bow_dic_path,
                                FLAGS_cam_calib_path,
                                !FLAGS_use_harris_feat,
                                FLAGS_iba_for_vio)) {
    LOG(ERROR) << "Init tracker failed";
    return -1;
  }

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
  // [Optional] Configure map loading and path follower
  bool is_path_follower_set = false;
  if (!FLAGS_pb_load.empty()) {
    if (!XP_TRACKER::load_map(FLAGS_pb_load)) {
      LOG(ERROR) << "Loading map failed";
    }
    if (!FLAGS_path_follower.empty()) {
      if (FLAGS_path_follower == "robot") {
        is_path_follower_set = XP_TRACKER::set_path_follower_robot();
      } else if (FLAGS_path_follower == "walk") {
        is_path_follower_set =
            XP_TRACKER::set_path_follower_walk_callback(path_follower_walk_callback);
      } else {
        LOG(ERROR) << "Not supported path follower type: " << FLAGS_path_follower;
        return -1;
      }
      if (!is_path_follower_set) {
        LOG(ERROR) << "set path follower: " << FLAGS_path_follower << " failed";
      }
    }
  }
  // The properties of cameras
  constexpr int img_h = 480;
  int img_w = 640;
#ifdef __linux__
  const char* env_display_p = std::getenv("DISPLAY");
  if (env_display_p == nullptr) {
    std::cout << "You are running headless OS. No X windows will be shown." << std::endl;
    FLAGS_no_display = true;
  }
#endif
  // set sensor data callback
  g_img_l_ptr.reset(new cv::Mat_<uchar>);
  cv::Mat_<uchar>& img_l = *g_img_l_ptr;
  if (FLAGS_show_simple || FLAGS_viz3d) {
    XP_TRACKER::set_stereo_images_callback(
      std::bind(&raw_sensor_img_callback,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));
  }
  #ifdef HAS_RECOGNITION

  #endif
  // set live recording path if not empty
  // [NOTE] Calling multiple times of run_tracker_MT will overwrite and only record the
  //        latest session.
  if (!FLAGS_record_path.empty()) {
    FLAGS_record_path = XP_TRACKER::set_record_path(FLAGS_record_path, FLAGS_cam_calib_path);
  }

  cv::Mat top_down_and_camera_canvas;
  cv::Mat camera_view_canvas;
  cv::Mat depth_view_canvas;
  cv::Mat path_follower_canvas;
  cv::Mat top_down_view_canvas;
  // init Viz3d in headless OS will crash the system
  float viz_cam_height = 1;  // in meter
#ifdef HAS_OPENCV_VIZ
  std::unique_ptr<PoseDrawer3D> pose_viewer_3d_ptr;
#endif

#ifdef SHOW_POSE_2D
  PoseDrawer2D pose_drawer;
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
        top_down_and_camera_canvas.create(img_h, img_w, CV_8UC3);
        // Draw trajectory directly on top of camera view
        top_down_view_canvas = top_down_and_camera_canvas;
        camera_view_canvas = top_down_and_camera_canvas;
        // In simple view mode, trajectory view is the same as left image,
        // so we do not clear canvas before plotting
        XP_TRACKER::set_top_view_canvas(&top_down_view_canvas, false);
        // If set_camera_view_canvas is not called
        // xp_tracker won't run feature point visualization thread
        // camera_view_canvas is modifed by image call back function
      } else {
#ifdef SHOW_POSE_2D
        // Use example 2D pose drawer
        top_down_and_camera_canvas.create(PoseDrawer2D::imageSize,
                                               PoseDrawer2D::imageSize + img_w, CV_8UC3);
        top_down_view_canvas =
          top_down_and_camera_canvas(cv::Rect(0, 0, PoseDrawer2D::imageSize,
                                                   PoseDrawer2D::imageSize));
#else
        // Use tracker drawer
        // do not show trajectory history
        if (FLAGS_show_both) {
          top_down_and_camera_canvas.create(img_h, img_h + img_w * 2, CV_8UC3);
        } else {
          top_down_and_camera_canvas.create(img_h, img_h + img_w, CV_8UC3);
        }
        top_down_view_canvas =
          top_down_and_camera_canvas(cv::Rect(0, 0, img_h, img_h));
        // Trajectory view is an independent view
        // Clear canvas before plotting
        XP_TRACKER::set_top_view_canvas(&top_down_view_canvas, true);
#endif  // SHOW_POSE_2D
        // set to black
        top_down_and_camera_canvas.setTo(cv::Vec3b(0, 0, 0));
        cv::putText(top_down_and_camera_canvas, "Waiting for initialization",
                    cv::Point(15, 15), cv::FONT_HERSHEY_COMPLEX,
                    0.5, cv::Scalar(255, 255, 255), 1);
        // Set camera_view_canvas as a sub-matrix of top_down_and_camera_canvas
        if (FLAGS_show_both) {
          camera_view_canvas =
            top_down_and_camera_canvas(cv::Rect(top_down_view_canvas.cols, 0,
                                                     img_w * 2, img_h));
        } else {
          camera_view_canvas =
            top_down_and_camera_canvas(cv::Rect(top_down_view_canvas.cols, 0,
                                                     img_w, img_h));
        }
        XP_TRACKER::set_camera_view_canvas(&camera_view_canvas);
        cv::namedWindow("Top View & Left Camera");
        cv::moveWindow("Top View & Left Camera", 0, 0);
      }  // FLAGS_show_simple
      cv::imshow("Top View & Left Camera", top_down_and_camera_canvas);
    }  // FLAGS_viz3d



    // Set depth canvas to start depth computation in tracker
    if (FLAGS_show_depth) {
      // depth is computed on half resolution
      // use CAMERA_IMG_HEIGHT CAMERA_IMG_WIDTH as dim to see distorted images
      // Set depth view canvas even if viz3d is used
      depth_view_canvas.create(img_h / 2, img_w / 2, CV_8UC3);
      XP_TRACKER::set_depth_view_canvas(&depth_view_canvas);
      if (!FLAGS_viz3d) {
        cv::namedWindow("Depth");
        cv::moveWindow("Depth", 500, 500);
      }
    }
    if (is_path_follower_set) {
      // eai is computed on half resolution
      // use CAMERA_IMG_HEIGHT CAMERA_IMG_WIDTH as dim to see distorted images
      path_follower_canvas.create(480, 1120, CV_8UC3);
      XP_TRACKER::set_path_follower_canvas(&path_follower_canvas);
      if (!FLAGS_viz3d) {
        cv::namedWindow("PathFollower");
        cv::moveWindow("PathFollower", 0, 600);
      }
    }
  }  // !FLAGS_no_display
  if (!XP_TRACKER::run_tracker_MT()) {
    LOG(ERROR) << "run_tracker_MT failed";
    return -1;
  }
  #ifdef HAS_RECOGNITION
  if (FLAGS_face_recognition || FLAGS_face_attribute || FLAGS_ocr) {
    XP_TRACKER::set_stereo_images_callback(
      std::bind(&raw_sensor_img_callback,
              std::placeholders::_1, std::placeholders::_2,
              std::placeholders::_3));
    read_api_keys();
  }
  #endif
  // cache for depth result
  cv::Mat_<cv::Vec3f> depth_result_img;
  if (!FLAGS_no_display) {
    char key_pressed = 0;
    // usleep(50000);
    int draw_counter = 0;
    while (!ESC_pressed) {
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
        } else {
          LOG(ERROR) << "XP_TRACKER::get_depth_img failed";
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
                                           *g_img_l_ptr,
                                           rig_xyz_mat,
                                           depth_result_img);
          }
        }
#endif
      } else {
#ifdef SHOW_POSE_2D  // defined in cmakelis
        float x, y, alpha;
        XP_TRACKER::get_tracker_latest_2d_pose(&x, &y, &alpha);
        pose_drawer.addPose(x, y, alpha);
        pose_drawer.drawTo(&top_down_view_canvas);
#else
        // top_down_view_canvas is drawn within tracker
#endif  // SHOW_POSE_2D
        if (FLAGS_show_simple) {
          if (g_img_l_ptr->rows != 0) {
            CHECK_EQ(g_img_l_ptr->size(), camera_view_canvas.size());
            cv::cvtColor(*g_img_l_ptr, camera_view_canvas, CV_GRAY2BGR);
          }
        }
        XP_TRACKER::draw_once();
        cv::imshow("Top View & Left Camera", top_down_and_camera_canvas);
#ifndef __CYGWIN__
        if (!FLAGS_vis_img_save_path.empty()) {
          cv::imwrite(FLAGS_vis_img_save_path, top_down_and_camera_canvas);
        }
#endif
        if (FLAGS_show_depth) {
          cv::imshow("Depth", depth_view_canvas);
        }
        // show path followerpose_drawer control view
        if (is_path_follower_set) {
          cv::imshow("PathFollower", path_follower_canvas);
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
        if (key_pressed == 27) {
          // viz3d may also set ESC_pressed
          ESC_pressed = true;
        } else if (key_pressed == 'A' || key_pressed == 'a' ||
                   key_pressed == 'B' || key_pressed == 'b' ||
                   key_pressed == 'C' || key_pressed == 'c' ||
                   key_pressed == 'D' || key_pressed == 'd' ||
                   key_pressed == 'E' || key_pressed == 'e') {
          if (FLAGS_path_follower.size() == 0) {
            // not in control mode
            // mark the latest KF
            // if there is alreayd a KF with this tag, force the new KF to share the same position
            // with the previous one. This is useful to correct path drift
            if (!XP_TRACKER::send_command_to_mapper("AddTagToLastKeyFrame", key_pressed)) {
              LOG(ERROR) << "send_command_to_mapper AddTagToLastKeyFrame failed";
            }
          } else {
            // in control mode
            // go to a frame with ID
            if (!XP_TRACKER::send_command_to_mapper("PathFollowerGoTo", key_pressed)) {
              LOG(ERROR) << "send_command_to_mapper PathFollowerGoTo failed";
            }
          }
        } else if (key_pressed == 'S' || key_pressed == 's') {
          XP_TRACKER::set_static(true);
        } else if (key_pressed == 'M' || key_pressed == 'm') {
          XP_TRACKER::set_static(false);
        #ifdef HAS_RECOGNITION
        } else if (key_pressed == 'r') {
          run_flag = !run_flag;
          if (run_flag) {
            std::thread(thread_apis).detach();
          }
        #endif
        } else if (key_pressed == -1) {
          // nothing is pressed
        } else {
          LOG(ERROR) << "Key " << static_cast<int32_t>(key_pressed)
                     << " is pressed but unknown";
        }
      }
      ++draw_counter;
    }
    cv::destroyAllWindows();
  } else {
    // FLAGS_no_display is set
    while (true) {
#ifndef __CYGWIN__
      if (!FLAGS_vis_img_save_path.empty()) {
        if (g_img_l_ptr->rows > 0) {
          cv::imwrite(FLAGS_vis_img_save_path, *g_img_l_ptr);
          usleep(100000);  // 10 Hz
        }
      }
#endif
    }
  }

  // Save the final mapper trajectory before exit
  if (FLAGS_record_path.empty()) {
    cv::imwrite("/tmp/final_traj.png", top_down_view_canvas);
  } else {
    cv::imwrite(FLAGS_record_path + "/final_traj.png", top_down_view_canvas);
  }

  if (!FLAGS_pb_save.empty()) {
    if (!XP_TRACKER::save_map(FLAGS_pb_save)) {
      LOG(ERROR) << "Save map failed " << FLAGS_pb_save;
    }
  }

  XP_TRACKER::stop_tracker();

  // Linux crashes at the end if release is not explicitly called
  g_img_l_ptr->release();
  return 0;
}
