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
/// \file

#include <xp_driver_interface.h>
// XP API
#include <XP/app_api/xp_tracker.h>
#include <XP/util/base64.h>
// Parsing flags and logging
#include <gflags/gflags.h>
#include <glog/logging.h>
// Added for the json-example
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <string>
#include <thread>

namespace live {
void XpDriverInterface::run() {
  if (interface_type_ == XP_sensor && xp_sensor_) {
    xp_sensor_->run();
  }
  if (interface_type_ == http_sensor) {
    thread_stream_images_pre_timestamp_ = std::chrono::steady_clock::now();
    std::thread server_thread([this]() {
      // fixme fix core dump problem here
      this->http_server_.start();
    });
  }
}

void XpDriverInterface::stop() {
  if (interface_type_ == XP_sensor && xp_sensor_) {
    xp_sensor_->stop();
  }
  if (interface_type_ == http_sensor) {
    http_server_.stop();
  }
}

bool XpDriverInterface::init_XP_sensor(const std::string &sensor_type,
                                       const bool auto_gain,
                                       const bool imu_from_image,
                                       const std::string sensor_dev_path,
                                       const std::string wb_mode) {
  if (interface_type_ != none) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = XP_sensor;
#ifdef __linux__
  if (sensor_type == "XP" ||
      sensor_type == "XP2" ||
      sensor_type == "XP3" ||
      sensor_type == "XPIRL" ||
      sensor_type == "XPIRL2" ||
      sensor_type == "ONE") {
    xp_sensor_.reset(new XPDRIVER::XpSensorMultithread(sensor_type,
                                                       auto_gain,
                                                       imu_from_image,
                                                       sensor_dev_path,
                                                       wb_mode));
    if (xp_sensor_) {
      return xp_sensor_->init();
    } else {
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

bool XpDriverInterface::init_rk_sensor() {
  if (interface_type_ != none) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = rk_sensor;
  LOG(FATAL) << "rk_sensor not supported yet!";
  return false;
}

bool XpDriverInterface::init_http_sensor() {
  if (interface_type_ != none) {
    LOG(ERROR) << "XpDriverInterface has already been inited.";
    return false;
  }
  interface_type_ = http_sensor;
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
          // calculate rate each 10 image counts
          if (this->stream_images_count_ > 10) {
            const auto now_ts = std::chrono::steady_clock::now();
            const int64 ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now_ts - this->thread_stream_images_pre_timestamp_).count();
            this->thread_stream_images_pre_timestamp_ = now_ts;
            if (ms != 0) {
              this->stream_images_rate_ = this->stream_images_count_ * 1000 / ms;
            } else {
              this->stream_images_rate_ = 0;
            }
            this->stream_images_count_ = 0;
            if (ms != 0) {
              this->pull_imu_rate_ = this->pull_imu_count_ * 1000 / ms;
            } else {
              this->pull_imu_rate_ = 0;
            }
            this->pull_imu_count_ = 0;
          }
          // todo(simulation) get images of img_l and img_r or imu data and base64_decode...
//          base64_decode(pt.get<string>(""));
//          XP_TRACKER::image_data_callback(img_l,
//                                          img_r,
//                                          ts_100us,
//                                          sys_time);  // Pass image into XP tracker
//          // Pass the left view for other processing
//          // [NOTE] this image_data_callback will only be triggered in the live sensor mode.
//          if (this->g_img_l_ptr) {
//            if (this->g_img_l_ptr->rows == 0) {
//              this->g_img_l_ptr->create(img_l.size(), img_l.type());
//            }
//            img_l.copyTo(*this->g_img_l_ptr);
//          }
//          XP_TRACKER::imu_data_callback(imu_data);  // Pass IMU data into XP tracker
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
  if (interface_type_ == XP_sensor && xp_sensor_) {
    return xp_sensor_->get_sensor_deviceid(device_id);
  } else if (interface_type_ == http_sensor) {
    LOG(FATAL) << "not implement get device id from http client yet";
    // todo(simulation) should get deviceid from http client
  } else {
    LOG(ERROR) << "you must init XP sensor first";
    return false;
  }
}

void XpDriverInterface::get_data_rate(float *img_rate, float *imu_rate) {
#ifdef __linux__
  if (interface_type_ == XP_sensor && xp_sensor_) {
    *img_rate = xp_sensor_->get_image_rate();
    *imu_rate = xp_sensor_->get_imu_rate();
    return;
  }
#endif  // __linux__
  if (interface_type_ == http_sensor) {
    *img_rate = stream_images_rate_;
    *imu_rate = pull_imu_rate_;
    return;
  }
  *img_rate = -1.f;
  *imu_rate = -1.f;
}

bool XpDriverInterface::register_data_callbacks() {
#ifdef __linux__
  if (interface_type_ == XP_sensor && xp_sensor_) {
    // [NOTE] Make sure no blocking operations within this function. This is the critical path
    // to pass data to our SLAM engine
    xp_sensor_->set_steady_image_callback(
        [this](const cv::Mat &img_l,
               const cv::Mat &img_r,
               const float ts_100us,
               const std::chrono::time_point<std::chrono::steady_clock> &sys_time) {
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
        });
    // [NOTE] Make sure no blocking operations within this function. This is the critical path
    // to pass data to our SLAM engine
    xp_sensor_->set_imu_data_callback([](const XPDRIVER::ImuData &imu_data) {
      XP_TRACKER::imu_data_callback(imu_data);  // Pass IMU data into XP tracker
    });
    return true;
  }
#endif  // __linux__
  if (interface_type_ == http_sensor) {
    LOG(INFO) << "using http_server, don't have to register";
    return true;
  }
  return false;
}

bool XpDriverInterface::auto_calib_load(XP::DuoCalibParam* calib_param) const {
  if (xp_sensor_) {
    if (interface_type_ == XP_sensor) {
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

}  // namespace live
