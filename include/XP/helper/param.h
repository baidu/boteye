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
#ifndef XP_INCLUDE_XP_HELPER_PARAM_H_
#define XP_INCLUDE_XP_HELPER_PARAM_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <limits>
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
  YAML::Node deserialize(const std::string& filename);

  YAML::Node base_node_;
};

class AlgorithmParam : public ParamBase {
 public:
  AlgorithmParam() {
    Nature.gravity = Eigen::Vector3f(0, 0, -9.799);
  }

  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;
  struct FeatDetParam_t {
    int request_feat_num = 70;
    int pyra_level = 2;
    int fast_det_thresh = 10;
    int uniform_radius = 40;
    float min_feature_distance_over_baseline_ratio = 3;
    float max_feature_distance_over_baseline_ratio = 3000;
    int feature_track_length_thresh = 25;
    float feature_track_dropout_rate = 0.3f;
  } FeatDetParam;
  struct Mapping_Mode_t {
    bool use_vio_orientation_as_absolute = false;
    bool compute_cov = true;
    bool fix_latest_rig_at_ba = true;
    bool detect_april_tag = true;
    bool modify_prescan_map = false;
    bool ignore_feat_on_floor = true;
    bool prescan_april_tags_only = false;
    bool disabled = false;
    bool fix_height_value = false;
  } MappingMode;
  struct Mapping_t {
    float max_feature_search_range = 20.f / 400.f;
    int min_keyrig_gap_num = 10;
    float min_keyrig_gap_time_sec = 0.5;
    int min_gen_local_map_gap_num = 10;
    float min_keyrig_dist = 0.05;
    float min_neighbour_bow_score = 0.1;
    float max_neighbour_rig_dis = 5.0;  // if a rig has high BOW score but larger spatial distance
                                        // than max_neighour_rig_dis, do not count it as key rig
    float orb_match_dist_thresh = 0.2 * 255;
    float orb_match_thresh_test_ratio = 0.8;
    int local_ba_min_movalble_rig_num = 1;
    int local_ba_max_movalble_rig_num = 5;
    int feat_det_num = 200;
    int feat_det_pyra_level = 2;
    float occupancy_vote_cell_size = 0.2;
    int feat_det_uniform_radius = 20;
    float run_dense_stereo_freq = 10;
    float camera_to_floor_height = 0.1;
    int max_vio_fail_count_tolerance = 10;
    int short_term_vio_fail_count = 3;
    int max_reloc_attempt = 5;
    float loop_detect_cooldown_sec = 10;
    float loop_close_cooldown_sec = 10;
    float max_lost_sec_before_use_vio_to_reset = 3;
    float smoothing_time_length = 0.1;
    int calib_verify_stereo_good_match_num = 20;
    std::string slave_det = "direct";
  } Mapping;
  struct Tracking_t {
    float max_feature_search_range = 10.f / 400.f;
    float orb_match_dist_thresh = 0.3 * 255;
    float orb_match_thresh_test_ratio = 0.9;
    float feature_uncertainty = 5;
    int imaging_FPS = 20;
    int imaging_exposure = 100;
    int imaging_gain = 100;
    int aec_index = 100;
    bool use_of_id = true;
    bool use_april_tag = false;
    std::string slave_det = "direct";
    bool undistort_before_vio = true;
  } Tracking;
  struct Reloc_t {
    float min_reloc_match_score = std::numeric_limits<float>::min();
    int min_3d_2d_num = 25;
    int min_ll_H_inlir_num = 10;
    float HScale_max = 2;
    float HScale_min = 0.5;
    float good_reproj_error = 0.025;  // 10 pixels with f = 400
    bool use_pnp = true;  // or relative pose
    float H_inlier_pixel_dis = 0.025;  // 10 pixels with f = 400
    float matches_area_ratio = 0.5;  // 0 - 1
    int H_max_plane_num = 3;
  };
  Reloc_t Reloc;
  struct Nature_t {
    Eigen::Vector3f gravity;
  } Nature;
};

class DuoCalibParam : public ParamBase {
 public:
  DuoCalibParam();
  ~DuoCalibParam() {}

  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;
  bool LoadCamCalibFromYaml(const std::string& filename);
  bool LoadImuCalibFromYaml(const std::string& filename);

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
    // the boundary of images in uv coordinate
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> lurd_lr;
    std::vector<cv::Mat> undistort_map_op1_lr;
    std::vector<cv::Mat> undistort_map_op2_lr;
    std::vector<cv::Matx33f> cv_undist_K_lr;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> undist_D_T_C_lr;
    cv::Matx44f Q;  // 4x4 convert disparity to depth. See cv::reprojectImageTo3D
    cv::Size img_size;
  } Camera;

  Eigen::Matrix3f C_R_B;
  Eigen::Vector3f C_p_B;
  std::string device_id;
  enum SensorType {
    UNKNOWN = 0,
    LI = 2,
    XP = 3,
    XP2 = 4,
    XP3 = 5,
  } sensor_type;
};

// Helper functions to load calibration parameters from calib.yaml
bool get_calib_file_from_device_id(const std::string& device_id,
                                   std::string* calib_file_ptr);
bool load_imu_calib_param(const std::string& device_id,
                          DuoCalibParam* duo_calib_param_ptr);

bool load_camera_calib_param(const std::string& calib_file,
                             XP::DuoCalibParam* duo_calib_param_ptr);
bool save_camera_calib_param(const std::string& calib_file,
                             const XP::DuoCalibParam& duo_calib_param);
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_PARAM_H_
