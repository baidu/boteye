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
#ifndef XP_INCLUDE_XP_HELPER_PARAM_H_
#define XP_INCLUDE_XP_HELPER_PARAM_H_

#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace XP {

class ParamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  ParamBase() {}
  virtual bool LoadFromYaml(const std::string& filename) = 0;
  virtual bool WriteToYaml(const std::string& filename) = 0;
  virtual bool LoadFromCvYaml(const std::string& filename) = 0;
  virtual bool WriteToCvYaml(const std::string& filename) = 0;

 protected:
  void serialize(const std::string& filename, const YAML::Emitter& emitter);
  void serializeToString(const YAML::Emitter& emitter, std::string* yaml_str);
  YAML::Node deserialize(const std::string& filename);
  YAML::Node deserializeFromString(const std::string& yaml_str);

  YAML::Node base_node_;
};

class DuoCalibParam : public ParamBase {
 public:
  DuoCalibParam();
  DuoCalibParam(const DuoCalibParam& rhs);  // copy constructor w/ deep copy
  DuoCalibParam& operator=(const DuoCalibParam& rhs);  // assign operator w/ deep copy
  ~DuoCalibParam() {}

  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;
  bool LoadFromString(const std::string& yaml_str);
  bool WriteToString(std::string* yaml_str);
  bool LoadCamCalibFromYaml(const std::string& filename);
  bool LoadImuCalibFromYaml(const std::string& filename);
  bool ConvertToHalfScale(void);

 protected:
  bool LoadCamCalibFromNode(const YAML::Node& node);
  bool LoadImuCalibFromNode(const YAML::Node& node);
  bool WriteToEmitter(YAML::Emitter* emitter);

 public:
  // init undistort map from camera K and distort
  bool initUndistortMap(const cv::Size& new_img_size);

  struct Imu_t {
    Eigen::Matrix3f accel_TK;
    Eigen::Vector3f accel_bias;
    Eigen::Matrix3f gyro_TK;
    Eigen::Vector3f gyro_bias;
    Eigen::Vector3f accel_noise_var;  // m / sec^2
    Eigen::Vector3f angv_noise_var;  // rad / sec
    Eigen::Matrix4f D_T_I;
    Eigen::Matrix4f undist_D_T_I;
  } Imu;

  struct Camera_t {
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> D_T_C_lr;
    std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f>> cameraK_lr;
    std::vector<cv::Matx33f> cv_camK_lr;
    std::vector<cv::Mat_<float>> cv_dist_coeff_lr;
    cv::Size img_size;
    // Variables below should be initialized by calling initUndistortMap
    // the boundary of images in uv coordinate
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> lurd_lr;
    std::vector<cv::Mat> undistort_map_op1_lr;
    std::vector<cv::Mat> undistort_map_op2_lr;
    std::vector<cv::Matx33f> cv_undist_K_lr;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> undist_D_T_C_lr;
    cv::Matx44f Q;  // 4x4 convert disparity to depth. See cv::reprojectImageTo3D
  } Camera;

  Eigen::Matrix3f C_R_B;
  Eigen::Vector3f C_p_B;
  std::string device_id;
  enum SensorType {
    UNKNOWN = 0,
    XP = 1,
    XP2 = 2,
    XP3 = 3,   // color sensor
    XPIRL = 4,
    XPIRL2 = 5,
    XPIRL3 = 6,
    XPIRL3_A = 7
  } sensor_type;
};

// Helper functions to load calibration parameters from calib.yaml
bool load_camera_calib_param(const std::string& calib_file,
                             XP::DuoCalibParam* duo_calib_param_ptr);
bool save_camera_calib_param(const std::string& calib_file,
                             const XP::DuoCalibParam& duo_calib_param);
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_PARAM_H_
