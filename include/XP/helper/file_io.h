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

#ifndef XP_INCLUDE_XP_HELPER_FILE_IO_H_
#define XP_INCLUDE_XP_HELPER_FILE_IO_H_

#include <XP/data_atom/basic_datatype.h>
#include <XP/helper/param.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <list>
#include <string>
#include <utility>
#include <vector>

namespace XP {

struct PlotData {
  float time_stamp;
  cv::Mat_<uchar> duo_raw_frame;
  std::vector<cv::KeyPoint> raw_key_pnts;
};

struct XpFeature {
  Eigen::Vector2f uv;
  int trace_id;
  cv::Mat desc;  // orb
};

class StereoFeatures {
 public:
  StereoFeatures() {
    stereo_features_.resize(2);
  }
  // 0 is left, 1 is right
  const std::vector<XpFeature>& lr(int lr) const { return stereo_features_[lr]; }
  std::vector<XpFeature>& lr_mutable(int lr) { return stereo_features_[lr]; }
  const std::vector<std::pair<int, int> >& id_to_org_id_vec() const { return id_to_org_id_vec_; }
  std::vector<std::pair<int, int> >& id_to_org_id_vec_mutable() { return id_to_org_id_vec_; }
  float timestamp() const { return timestamp_; }
  void set_timestamp(float timestamp) { timestamp_ = timestamp; }
 private:
  float timestamp_ = 0;
  std::vector<std::vector<XpFeature> > stereo_features_;
  std::vector<std::pair<int, int> > id_to_org_id_vec_;
};

size_t load_xp_imu_data(const std::string& imu_file_str,
                        std::list<XP::ImuData>* imu_samples_ptr,
                        const bool from_100us_to_sec);

bool load_xp_rig(const std::vector<std::string>& orb_yml_lr,
                 XP::StereoFeatures* stereo_feats_ptr);

size_t load_xp_orb_detection(const std::string& data_folder_str,
                             const std::string& det_folder_suffix,
                             const std::string& yml_list_file_str,
                             std::vector<StereoFeatures>* stereo_pnts_ptr,
                             std::vector<std::vector<std::string> >* orb_yml_files_ptr);
bool load_single_orb_detection(const std::string& yml_file_str,
                               std::vector<cv::KeyPoint>* keypoits,
                               cv::Mat* orb_feats);
// get the time stamp in image names
// data_folder_path/l/XXX.png
// data_folder_path/r/XXX.png
bool get_image_names_in_record_folder(const std::string& data_folder_path,
                                      std::vector<std::string>* image_stem_str,
                                      const std::string& img_suffix = "png");
bool load_calib_yaml_from_record_folder(const std::string& folder_path,
                                        XP::DuoCalibParam* calib_param);
}  // namespace XP

#endif  // XP_INCLUDE_XP_HELPER_FILE_IO_H_
