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

#include <xp_driver_interface.h>
// XP API
#include <XP/app_api/xp_tracker.h>
#include <XP/helper/file_io.h>
#include <XP/util/base64.h>
// Added for the json-example
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <string>
#include <thread>
#include <list>
#include <vector>

using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::milliseconds;
using std::chrono::microseconds;

namespace live {
void XpDriverInterface::run() {
  if (interface_type_ == XP_SENSOR && xp_sensor_) {
    xp_sensor_->run();
  }
  if (interface_type_ == HTTP_SENSOR) {
    std::thread([this]() {
      // TODO(yiming) fix core dump problem here
      this->http_server_.start();
    });
  }
  if (interface_type_ == RECORD_LOADER) {
#ifdef __CYGWIN__
    // TODO(sid bao) cywin support
    LOG(ERROR) << "cygwin not supported yet";
    exit(1);
#else
    record_loader_thread_ =
        std::thread(&XpDriverInterface::thread_record_loader, this, video_dev_file_);
#endif
  }
  thread_stream_images_pre_timestamp_ = steady_clock::now();
  thread_pull_imu_pre_timestamp_ = thread_stream_images_pre_timestamp_;
}

void XpDriverInterface::stop() {
  if (interface_type_ == XP_SENSOR && xp_sensor_) {
    xp_sensor_->stop();
  }
  if (interface_type_ == HTTP_SENSOR) {
    http_server_.stop();
  }
  if (interface_type_ == RECORD_LOADER) {
    is_loader_running_ = false;
    if (record_loader_thread_.joinable()) {
      record_loader_thread_.join();
    }
  }
}

bool XpDriverInterface::init_XP_sensor(const std::string &sensor_type,
                                       const bool auto_gain,
                                       const bool imu_from_image,
                                       const std::string dev_path,
                                       const std::string sensor_id,
                                       const std::string wb_mode) {
  if (interface_type_ != NONE) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = XP_SENSOR;
#ifdef __linux__
  if (sensor_type == "XP" ||
      sensor_type == "XP2" ||
      sensor_type == "XP3" ||
      sensor_type == "XPIRL" ||
      sensor_type == "XPIRL2" ||
      sensor_type == "BoteyeOne") {
    if (XPDRIVER::init_sensor(sensor_type,
                              auto_gain,
                              imu_from_image,
                              dev_path,
                              sensor_id,
                              wb_mode,
                              xp_sensor_)) {
      // [NOTE] Make sure no blocking operations within this function. This is the critical path
      // to pass data to our SLAM engine
      xp_sensor_->
          set_steady_image_callback([this](const cv::Mat &img_l,
                                           const cv::Mat &img_r,
                                           const float ts_100us,
                                           const std::chrono::time_point<steady_clock> &sys_time) {
        XP_TRACKER::image_data_callback(img_l,
                                        img_r,
                                        ts_100us,
                                        sys_time);  // Pass image into XP tracker
#ifdef HAS_ROS
        if (ros::ok()) {
          std_msgs::Header img_header;
          img_header.seq = this->img_seq_ID_++;
          img_header.frame_id = "device";
          img_header.stamp = ros::Time::now();
          std::string encoding = img_r.channels() == 1 ? sensor_msgs::image_encodings::MONO8
                                                       : sensor_msgs::image_encodings::BGR8;
          app_tracking::XpDriverImage msg;
          sensor_msgs::ImagePtr
              msg_l = cv_bridge::CvImage(img_header, encoding, img_l.clone()).toImageMsg();
          sensor_msgs::ImagePtr
              msg_r = cv_bridge::CvImage(img_header, encoding, img_r.clone()).toImageMsg();
          msg.img_l = *msg_l;
          msg.img_r = *msg_r;
          msg.ts_100us = ts_100us;
          msg.nanoseconds = sys_time.time_since_epoch().count();
          image_pub.publish(msg);
          ros::spinOnce();
        }
#endif
        // Pass the left view for other processing
        // [NOTE] this image_data_callback will only be triggered in the live sensor mode.
        if (this->g_img_l_ptr) {
          if (this->g_img_l_ptr->rows == 0) {
            this->g_img_l_ptr->create(img_l.size(), img_l.type());
          }
          img_l.copyTo(*this->g_img_l_ptr);
        }
      });
      // [NOTE] Make sure no blocking operations within this function. This is the critical path
      // to pass data to our SLAM engine
      xp_sensor_->set_imu_data_callback([this](const XPDRIVER::ImuData &imu_data) {
        XP_TRACKER::imu_data_callback(imu_data);  // Pass IMU data into XP tracker
#ifdef HAS_ROS
        if (ros::ok()) {
          app_tracking::XpDriverImu msg;
          // accel is in m/s^2
          // angv is in rad/s
          msg.timestamp = imu_data.time_stamp;
          msg.linear_acceleration_x = imu_data.accel[0];
          msg.linear_acceleration_y = imu_data.accel[1];
          msg.linear_acceleration_z = imu_data.accel[2];
          msg.angular_velocity_x = imu_data.ang_v[0];
          msg.angular_velocity_y = imu_data.ang_v[1];
          msg.angular_velocity_z = imu_data.ang_v[2];
          IMU_pub.publish(msg);
          ros::spinOnce();
        }
#endif
      });
      return true;
    } else {
      LOG(ERROR) << "Sensor init failed!";
      return false;
    }
  } else {
    LOG(ERROR) << "sensor type " << sensor_type << " NOT supported!";
    return false;
  }
#else
  LOG(ERROR) << "Only support Linux for now";
  return false;
#endif  // __linux__
}

#ifdef HAS_ROS
void XpDriverInterface::imageRosCb(const app_tracking::XpDriverImagePtr &msg) {
  namespace enc = sensor_msgs::image_encodings;
  cv_bridge::CvImageConstPtr cv_ptr_l;
  cv_bridge::CvImageConstPtr cv_ptr_r;
  try {
    if (enc::isColor(msg->img_l.encoding)) {
      cv_ptr_l = cv_bridge::toCvCopy(msg->img_l, enc::BGR8);
      cv_ptr_r = cv_bridge::toCvCopy(msg->img_r, enc::BGR8);
    } else {
      cv_ptr_l = cv_bridge::toCvCopy(msg->img_l, enc::MONO8);
      cv_ptr_r = cv_bridge::toCvCopy(msg->img_r, enc::MONO8);
    }
    ++this->stream_images_count_;
    calculate_img_imu_rate();
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  XP_TRACKER::
  image_data_callback(cv_ptr_l->image,
                      cv_ptr_r->image,
                      msg->ts_100us,
                      std::chrono::time_point<steady_clock>(nanoseconds(msg->nanoseconds)));
}

void XpDriverInterface::imuRosCb(const app_tracking::XpDriverImuPtr &msg) {
  XPDRIVER::ImuData imu_data;
  imu_data.time_stamp = msg->timestamp;
  imu_data.accel[0] = msg->linear_acceleration_x;
  imu_data.accel[1] = msg->linear_acceleration_y;
  imu_data.accel[2] = msg->linear_acceleration_z;
  imu_data.ang_v[0] = msg->angular_velocity_x;
  imu_data.ang_v[1] = msg->angular_velocity_y;
  imu_data.ang_v[2] = msg->angular_velocity_z;
  ++this->pull_imu_count_;
  XP_TRACKER::imu_data_callback(imu_data);  // Pass IMU data into XP tracker
}
#endif

bool XpDriverInterface::init_ros_subscriber() {
  if (interface_type_ != NONE) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
#ifdef HAS_ROS
  interface_type_ = ROS_SUBSCRIBER;
  std::thread([this]() {
    auto image_sub = nh_.subscribe("sensor/image", 125, &XpDriverInterface::imageRosCb, this);
    auto IMU_sub = nh_.subscribe("sensor/IMU_data", 500, &XpDriverInterface::imuRosCb, this);
    while (ros::ok()) {
      ros::spinOnce();
    }
  }).detach();
  return true;
#else
  LOG(ERROR) << "You must have ros-kinetic installed on your robot!";
  return false;
#endif
}

bool XpDriverInterface::init_data_loader(const std::string &folder_path) {
  if (interface_type_ != NONE) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = RECORD_LOADER;
  if (folder_path.empty()) {
    LOG(ERROR) << "folder_path is empty";
    return false;
  }
  video_dev_file_ = folder_path;
  return true;
}

void XpDriverInterface::thread_record_loader(const std::string &data_full_path) {
#ifndef __DUO_VIO_TRACKER_NO_DEBUG__
  VLOG(1) << "======== thread_record_loader starts";
#endif
  // Load IMU data
  std::string imu_data_file_name = data_full_path + "/imu_data.txt";
  std::list<XP::ImuData> imu_samples;
  if (XP::load_xp_imu_data(imu_data_file_name, &imu_samples, false/*from_100us_to_sec*/) == 0) {
    LOG(ERROR) << "Error in imu_data_file_name";
    return;
  }
  // get image list
  std::vector<std::string> image_timestamps;
  if (!XP::get_image_names_in_record_folder(data_full_path, &image_timestamps)) {
    LOG(ERROR) << "Error in get_image_names_in_record_folder";
    return;
  }
  std::list<XP::ImuData>::iterator it_imu_samples = imu_samples.begin();
  size_t it_img_timestamp = 2;  // Start from the 3nd img to have imu measurements in front
  const std::chrono::time_point<steady_clock> simulation_start_tp = steady_clock::now();
  is_loader_running_ = true;
  while (is_loader_running_) {
    const int64 simulation_passed_us =
        duration_cast<microseconds>(steady_clock::now() - simulation_start_tp).count();
    const double simulation_passed_100us = simulation_passed_us / 1e2;
    const double simulation_passed_time = simulation_passed_us / 1e6;
    // out IMU data
    while (it_imu_samples != imu_samples.end()) {
      if (it_imu_samples->time_stamp < simulation_passed_100us) {
        XPDRIVER::ImuData imu_data;
        imu_data.time_stamp = it_imu_samples->time_stamp;
        imu_data.accel[0] = it_imu_samples->accel[0];
        imu_data.accel[1] = it_imu_samples->accel[1];
        imu_data.accel[2] = it_imu_samples->accel[2];
        imu_data.ang_v[0] = it_imu_samples->ang_v[0];
        imu_data.ang_v[1] = it_imu_samples->ang_v[1];
        imu_data.ang_v[2] = it_imu_samples->ang_v[2];
        XP_TRACKER::imu_data_callback(imu_data);
        ++it_imu_samples;
        // for statistics
        ++pull_imu_count_;
      } else {
        break;
      }
    }
    // image data
    while (it_img_timestamp < image_timestamps.size()) {
      const int img_timestamp_100us = boost::lexical_cast<int>(image_timestamps[it_img_timestamp]);
      const double img_timestamp = img_timestamp_100us / 1e4;
      if (img_timestamp < simulation_passed_time - 0.05) {  // simulate image delay
        // TODO(mingyu): Force to load images as gray for now
        cv::Mat img_l =
            cv::imread(data_full_path + "/l/" + image_timestamps[it_img_timestamp] + ".png",
                       CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat img_r =
            cv::imread(data_full_path + "/r/" + image_timestamps[it_img_timestamp] + ".png",
                       CV_LOAD_IMAGE_GRAYSCALE);
        XP_TRACKER::image_data_callback(img_l, img_r, img_timestamp_100us, steady_clock::now());
      } else {
        // img_timestamp >= simulation_passed_time
        break;
      }  // if (img_timestamp < simulation_passed_time)
      it_img_timestamp++;
      ++stream_images_count_;
    }
    calculate_img_imu_rate();
    // check do we hit the end of the data recording
    if (it_imu_samples == imu_samples.end() || it_img_timestamp == image_timestamps.size()) {
      break;
    }
    // sleep for a short while to prevent thread jamming
    usleep(100);
  }

  // Kill the normal process
  XP_TRACKER::stop_tracker("XpDriverInterface::thread_record_loader");
  std::cout << "Data loader: end of data" << std::endl;
}

bool XpDriverInterface::init_http_sensor() {
  if (interface_type_ != NONE) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = HTTP_SENSOR;
  http_server_.config.port = 12080;
  http_server_.on_error =
      [](std::shared_ptr<HttpServer::Request> /*request*/, const SimpleWeb::error_code &ec) {
        std::cerr << "Connection interrupted: " << ec << std::endl;
      };

  http_server_.resource["^/json$"]["POST"] =
      [this](std::shared_ptr<HttpServer::Response> response,
             std::shared_ptr<HttpServer::Request> request) {
        try {
          boost::property_tree::ptree pt;
          boost::property_tree::read_json(request->content, pt);

          auto rcv = pt.get<std::string>("status") + " " +
              pt.get<std::string>("position_x") + " " +
              pt.get<std::string>("position_y") + " " +
              pt.get<std::string>("position_z") + " " +
              pt.get<std::string>("time") + " " +
              pt.get<std::string>("width") + " " +
              pt.get<std::string>("height") + "\n";

          bool request_type_image = true;
          if (request_type_image) {
            ++this->stream_images_count_;
          } else {
            ++this->pull_imu_count_;
          }
          calculate_img_imu_rate();
          // todo(simulation) get images of img_l and img_r or imu data and base64_decode...
          /*
          base64_decode(pt.get<string>(""));
          XP_TRACKER::image_data_callback(img_l,
                                          img_r,
                                          ts_100us,
                                          sys_time);  // Pass image into XP tracker
          // Pass the left view for other processing
          // [NOTE] this image_data_callback will only be triggered in the live sensor mode.
          if (this->g_img_l_ptr) {
            if (this->g_img_l_ptr->rows == 0) {
              this->g_img_l_ptr->create(img_l.size(), img_l.type());
            }
            img_l.copyTo(*this->g_img_l_ptr);
          }
          XP_TRACKER::imu_data_callback(imu_data);  // Pass IMU data into XP tracker
          */
          *response << "HTTP/1.1 200 OK\r\n"
                    << "Content-Length: " << pt.get<std::string>("time").length() << "\r\n\r\n"
                    << pt.get<std::string>("time");
        }
        catch (const std::exception &e) {
          *response << "HTTP/1.1 400 Bad Request\r\nContent-Length: "
                    << strlen(e.what()) << "\r\n\r\n"
                    << e.what();
        }
      };
  return true;
}

bool XpDriverInterface::get_sensor_deviceid(std::string *device_id) {
  if (interface_type_ == XP_SENSOR && xp_sensor_) {
    return xp_sensor_->get_sensor_deviceid(device_id);
  } else if (interface_type_ == HTTP_SENSOR) {
    LOG(FATAL) << "not implement get device id from http client yet";
    // todo(simulation) should get deviceid from http client
  } else {
    LOG(ERROR) << "you must init XP sensor first";
    return false;
  }
}

void XpDriverInterface::get_data_rate(float *img_rate, float *imu_rate) {
#ifdef __linux__
  if (interface_type_ == XP_SENSOR && xp_sensor_) {
    *img_rate = xp_sensor_->get_image_rate();
    *imu_rate = xp_sensor_->get_imu_rate();
    return;
  }
#endif  // __linux__
  *img_rate = stream_images_rate_;
  *imu_rate = pull_imu_rate_;
}

// TODO(huyuexiang) wrap "auto_calib_load" in driver util
bool XpDriverInterface::auto_calib_load(XP::DuoCalibParam *calib_param) const {
  if (xp_sensor_) {
    if (interface_type_ == XP_SENSOR) {
      std::string calib_str;
      LOG(INFO) << "Loading calib file from sensor...";
      if (xp_sensor_->get_calib_from_sensor(&calib_str)) {
        calib_param->LoadFromString(calib_str);
        LOG(INFO) << "Load calib file successfully!";
        std::cout << "Load calib file from sensor successfully\n";
        return true;
      } else {
        LOG(ERROR) << "Failed to load calib from sensor";
        return false;
      }
    } else {
      LOG(ERROR) << "Only support load calib from XP sensor";
      return false;
    }
  } else {
    LOG(ERROR) << "Must init sensor first!";
    return false;
  }
}

bool XpDriverInterface::load_calib_from_folder(const std::string &load_path,
                                               XP::DuoCalibParam *calib_param) const {
  if (!load_calib_yaml_from_record_folder(load_path, calib_param)) {
    LOG(ERROR) << "Cannot find calib yaml in folder path: " << load_path;
    return false;
  }
  return true;
}

bool XpDriverInterface::load_calib_from_file(const std::string &calib_file,
                                             XP::DuoCalibParam *calib_param) const {
  if (calib_file.empty()) {
    LOG(ERROR) << "You must provide calib file by setting -calib_file";
    return false;
  }
  if (!calib_param->LoadFromYaml(calib_file)) {
    LOG(ERROR) << "Failed to load calib from " << calib_file;
    return false;
  }
  return true;
}

inline void XpDriverInterface::calculate_img_imu_rate() {
  if (pull_imu_count_ > 50) {
    const int64 ms =
        duration_cast<milliseconds>(steady_clock::now() - thread_pull_imu_pre_timestamp_).count();
    thread_pull_imu_pre_timestamp_ = steady_clock::now();
    pull_imu_rate_ = pull_imu_count_ * 1000 / ms;
    pull_imu_count_ = 0;
  }
  // for statistics
  if (stream_images_count_ > 10) {
    const int64 ms = duration_cast<milliseconds>(
        steady_clock::now() - thread_stream_images_pre_timestamp_).count();
    thread_stream_images_pre_timestamp_ = steady_clock::now();
    stream_images_rate_ = stream_images_count_ * 1000 / ms;
    stream_images_count_ = 0;
  }
}

}  // namespace live
