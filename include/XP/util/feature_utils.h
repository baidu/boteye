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
#ifndef XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_

#include <XP/helper/param.h>
#include <glog/logging.h>
#include <opencv2/video/tracking.hpp>

#include <map>
#include <mutex>
#include <random>
#include <vector>
#include <unordered_map>
#include <utility>
#include <string>

namespace XP {

// this typedef shows up twice, seems like nothing bad happens.
// try this
#ifndef XP_TYPEDEF_FRAME_WITH_FEATURE_LIST_
#define XP_TYPEDEF_FRAME_WITH_FEATURE_LIST_
typedef std::pair<std::vector<cv::KeyPoint>, cv::Mat> FrameWithFeatureList;
#endif

class IdGenerator {
 public:
  IdGenerator() : id_(0) {}
  int get() {
    std::lock_guard<std::mutex> lock(id_mutex_);
    ++id_;
    // wrap around
    if (id_ <= 0) id_ = 1;  // 0 indicates an invalid id!
    return id_;
  }
  void reset(int id) { id_ = id; }
 private:
  std::mutex id_mutex_;
  int id_;
};

// This class performs feature (re)detection + feature propagation with optical flow by
// only considering the master view
class FeatureTrackDetector {
 public:
  struct FeatureTrack {
  explicit FeatureTrack(const cv::Point2f pt) : length(1), isActive(true), point(pt) {}
    int length;
    bool isActive;
    cv::Point2f point;
    std::vector<int> keyframes_id;  // keyframes included in this feature track.
    // descriptor. 32 is for ORB. maybe we don't need this,
    // since descriptors are stored in FrameWithFeatureList struct
    // in the covisgraph.
    uint8_t desc_[32];
  };

  FeatureTrackDetector(const int length_thres,
                       const float drop_rate,
                       const bool use_fast,  // True: fast; False: ShiTomasi
                       const int uniform_radius,  // <= 5 means no uniformity suppression
                       const cv::Size& img_size);
  // flag keep_dead_feat_tracks is set to false as default,
  // so that present calls to optical_flow_and_detect() won't fail if it's not given
  bool optical_flow_and_detect(const cv::Mat_<uchar>& mask,
                               const cv::Mat& pre_image_orb_feature,
                               const std::vector<cv::KeyPoint>& prev_img_kpts,
                               int request_feat_num,
                               int pyra_level_det,
                               int fast_thresh,
                               std::vector<cv::KeyPoint>* key_pnts_ptr,
                               cv::Mat* orb_feat_ptr = nullptr,
                               // Before OF process, add init_pixel_shift to each (x, y) of
                               // pre_image_keypoints as the initial value
                               const cv::Vec2f& init_pixel_shift = cv::Vec2f(0, 0),
                               // provide the following info enables feature location prediction
                               const cv::Matx33f* K_ptr = nullptr,
                               const cv::Mat_<float>* dist_ptr = nullptr,
                               const cv::Matx33f* old_R_new_ptr = nullptr,
                               const bool absolute_static = false,
                               const bool keep_dead_feat_tracks = false);
  // This function is exclusively for re_detect decision and operation.
  // Supposed to be called after the initial OF feature propagation from last frame,
  // and the hybrid feature propagation from related key frames, consecutively.
  // Key_pnts_ptr and orb_feat_ptr are input/output parameters,
  // which are updated by the function.
  // flag keep_dead_feat_tracks is to control whether to keep dead feature tracks
  // after each propagation.
  bool re_detect(int request_feat_num,
                 int pyra_level_det,
                 int fast_thresh,
                 const cv::Mat& orb_feat_from_prop,
                 std::vector<cv::KeyPoint>* key_pnts_ptr,
                 cv::Mat* orb_feat_ptr,
                 const bool keep_dead_feat_tracks = false);
  int find_candidate_keyframes(const XP::FrameWithFeatureList&,
                               std::unordered_map<int, int>* keyframe_hitlist);
  bool hybrid_match(const cv::Mat& img_in_smooth,
                    const cv::Mat& keyframe_smooth,
                    std::vector<cv::KeyPoint>* key_pnts_ptr,
                    cv::Mat* orb_feat_ptr,
                    const std::vector<cv::KeyPoint>& key_pnts_all,
                    const cv::Mat& orb_feat_all,
                    const XP::FrameWithFeatureList&,
                    const XP::FrameWithFeatureList&,
                    const cv::Matx33f* K_ptr,
                    const cv::Mat_<float>* dist_ptr,
                    const int keyframe_id,
                    const int current_id,
                    int* weight,
                    int max_dist_epi,
                    int method_f,
                    const std::vector<int>& debug_frames,
                    std::vector<int>* outlier_ids = nullptr);

  void update_img_pyramids() {
    curr_img_pyramids_.swap(prev_img_pyramids_);
    curr_pyramids_buffer_.swap(prev_pyramids_buffer_);
  }
  bool detect(const cv::Mat_<uchar>& mask,
              int request_feat_num,
              int pyra_levels,  // Total pyramid levels, including the base image
              int fast_thresh,
              std::vector<cv::KeyPoint>* key_pnts_ptr,
              cv::Mat* orb_feat_ptr);
  bool detect_and_match(const cv::Mat& img_in_smooth,
                        const cv::Mat_<uchar>& mask,
                        int request_feat_num,
                        const cv::Mat& pre_image_orb_feature,
                        const std::vector<cv::KeyPoint>& pre_image_kpts,
                        int pyra_levels,  // Total pyramid levels, including the base image
                        int fast_thresh,
                        std::vector<cv::KeyPoint>* key_pnts_ptr,
                        cv::Mat* orb_feat_ptr);

  inline size_t feature_tracks_number() const { return feature_tracks_map_.size(); }
  int add_new_feature_track(const cv::Point2f pt);  // Return the added feature track id
  void mark_all_feature_tracks_dead();
  void filter_static_features(std::vector<cv::KeyPoint>* key_pnts);
  void update_feature_tracks(std::vector<cv::KeyPoint>* key_pnts);
  void flush_feature_tracks(const std::vector<cv::KeyPoint>& key_pnts);
  void update_feature_tracks_hist(bool update_active_fts);
  void output_feature_tracks_hist(std::string frame_ts);
  inline const std::vector<int>& get_ft_hist() const {
    return feature_tracks_hist_;
  }
  inline std::vector<std::pair<int, int> > get_id_to_org_id_vec() const {
    return id_to_org_id_vec_;
  }
  // Add current keyframe id (and descriptor?) to active feature tracks
  // maybe we don't need to update descriptor for now
  void update_featuretrack_history(const int keyframe_id);

 protected:
  // Once feature track exceeds this threshold, we break this feature track randomly
  // with drop_rate.
  // [NOTE] Use a negative drop rate for no drop out
  int length_threshold_;
  float drop_rate_;
  bool use_fast_;
  int uniform_radius_;
  IdGenerator id_generator_;

  // A very simple-minded data structure to bookkeep feature track ids and lengths
  // We will have at most request_feat_num active feature tracks
  std::map<int,  FeatureTrack> feature_tracks_map_;

  // A histogram for lengths of feature tracks
  // The length of each feature track is added to the histogram before it's dead
  std::vector<int> feature_tracks_hist_;

  // A mask to hold the mask of the union of input mask and optical flow feats mask
  cv::Mat_<uchar> mask_with_of_out_;

  // A vector of <class_id, original_class_id> pair for feature tracks that are randomly
  // selected to be dropped during update_feature_tracks process
  std::vector<std::pair<int, int> > id_to_org_id_vec_;

 private:
  // Random generator
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;

 private:
  // For XP optical flow
  std::shared_ptr<uchar> prev_pyramids_buffer_;  // buffer for storing previous pyramids
  std::shared_ptr<uchar> curr_pyramids_buffer_;  // buffer for storing current pyramids
  std::vector<cv::Mat> prev_img_pyramids_;
  std::vector<cv::Mat> curr_img_pyramids_;

 public:
  static constexpr int kMaxPyraLevelOF = 3;
  uchar* const get_prev_pyramids_buffer() const {
    return prev_pyramids_buffer_.get();
  }

  uchar* const get_curr_pyramids_buffer() const {
    return curr_pyramids_buffer_.get();
  }
  std::vector<cv::Mat>& get_prev_img_pyramids() {
    return prev_img_pyramids_;
  }
  std::vector<cv::Mat>& get_curr_img_pyramids() {
    return curr_img_pyramids_;
  }
  enum {
    BUILD_TO_CURR = 0,
    BUILD_TO_PREV = 1
  };
  void build_img_pyramids(const cv::Mat& img_in_smooth, int build_type = BUILD_TO_CURR);
  void set_mask_with_of_out(const cv::Mat_<uchar>& mask_with_of_out) {
    mask_with_of_out_ = mask_with_of_out.clone();
  }
};

// detect features on slave img
// assume slave image is lr = 1, master is lr = 0
// copy the kp class_id from master to slave points
#ifndef __FEATURE_UTILS_NO_DEBUG__
#undef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
#endif
class SlaveImgFeatureDetector {
 public:
  enum DetectSlaveFeatureType {
    EPIPOLAR_LINE_SEARCH = 0,
  };
  explicit SlaveImgFeatureDetector(int block_size = 8,  // block matching size. Multiple of 4
                                   DetectSlaveFeatureType method =
                                   DetectSlaveFeatureType::EPIPOLAR_LINE_SEARCH,
                                   float min_feature_distance_over_baseline_ratio = 3,
                                   float max_feature_distance_over_baseline_ratio = 3000);
  ~SlaveImgFeatureDetector();
  bool detect_features_on_slave_img(const cv::Mat& master_image,
                                    const cv::Mat& slave_image,
                                    const std::vector<cv::KeyPoint>& master_kps,
                                    const DuoCalibParam& duo_calib_param,
                                    std::vector<cv::KeyPoint>* slave_kps_ptr,
                                    cv::Mat* slave_orb_feat_ptr = nullptr,
                                    int max_pixel_val_diff = 15);

 private:
  bool detect_features_on_slave_img_helper(const cv::Mat& master_image,
                                           const cv::Mat& slave_image,
                                           const DuoCalibParam& duo_calib_param,
                                           int max_pixel_val_diff,
                                           int master_x,
                                           int master_y,
                                           const cv::Mat_<float>& s_R_m,
                                           const cv::Mat_<float>& s_t_m,
                                           const cv::Mat_<float>& pnt_master,
                                           const cv::Mat_<float>& search_range,
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
                                           cv::Mat* slave_image_debug_ptr,
#endif
                                           int* min_patch_diff2_ptr,
                                           int* second_min_patch_diff2_ptr,
                                           int* best_slave_x_ptr,
                                           int* best_slave_y_ptr,
                                           int* best_search_dist_id_ptr,
                                           int* second_best_search_dist_id_ptr);

  // the most light data structure
  float* gaussion_weights_;
  float gaussion_weight_sum_;
  const int block_size_;
  const int half_block_size_;
  const DetectSlaveFeatureType method_;
  const float min_feature_distance_over_baseline_ratio_;
  const float max_feature_distance_over_baseline_ratio_;
};

class ImgFeaturePropagatorImpl;  // Forward declaration
class ImgFeaturePropagator {
 public:
  ImgFeaturePropagator(const Eigen::Matrix3f& cur_camK,
                       const Eigen::Matrix3f& ref_camK,
                       const cv::Mat_<float>& cur_cv_dist_coeff,
                       const cv::Mat_<float>& ref_cv_dist_coeff,
                       const cv::Mat_<uchar>& cur_mask,
                       float min_feature_distance_over_baseline_ratio,
                       float max_feature_distance_over_baseline_ratio);
  ~ImgFeaturePropagator();

  bool PropagateFeatures(const cv::Mat& cur_img,
                         const std::vector<cv::Mat>& ref_pyra_imgs,
                         const std::vector<cv::KeyPoint>& ref_keypoints,
                         const Eigen::Matrix4f& T_ref_cur,
                         const bool use_pyra_direct_matcher,
                         std::vector<cv::KeyPoint>* cur_keypoints,
                         cv::Mat* cur_orb_features = nullptr,
                         const bool draw_debug = false);

 private:
  ImgFeaturePropagatorImpl* impl_;
};

// Utility functions related to feature detections
bool generate_cam_mask(const cv::Matx33f& K,
                       const cv::Mat_<float>& dist_coeffs,
                       const cv::Size& mask_size,
                       cv::Mat_<uchar>* cam_mask,
                       float* fov_deg);

bool detect_orb_features(const std::vector<cv::Mat>& img_pyramids,
                         const std::vector<cv::Mat_<uchar> >& mask_pyramids,
                         int request_feat_num,
                         int det_pyra_level,  // The specific pyramid level to do detection
                         int fast_thresh,
                         bool use_fast,  // or TomasShi
                         int enforce_uniformity_radius,  // less than 5 means no enforcement
                         std::vector<cv::KeyPoint>* key_pnts_ptr,
                         cv::Mat* orb_feat_ptr,
                         FeatureTrackDetector* feat_track_detector = nullptr,
                         float refine_harris_threshold = -1.f);

// This is the OLD interface to do feature detection with an input raw image, and
// internally compute the image pyramids and then call the NEW interface above.
bool detect_orb_features(const cv::Mat& img_in_raw,
                         const cv::Mat_<uchar>& mask,
                         int request_feat_num,
                         int pyra_levels,  // Total pyramid levels (including the base pyr0)
                         int fast_thresh,
                         bool use_fast,  // or TomasShi
                         int enforce_uniformity_radius,  // less than 5 means no enforcement
                         std::vector<cv::KeyPoint>* key_pnts_ptr,
                         cv::Mat* orb_feat_ptr,
                         FeatureTrackDetector* feat_track_detector = nullptr,
                         float refine_harris_threshold = -1.f);

// [NOTE] We keep this NON-pyramid general interface to support slave_det_mode = OF
// This function is a wrapper of the pyramid version below.
void propagate_with_optical_flow(const cv::Mat& img_in_smooth,
                                 const cv::Mat_<uchar>& mask,
                                 const cv::Mat& pre_image,
                                 const cv::Mat& pre_image_orb_feature,
                                 const std::vector<cv::KeyPoint>& pre_image_keypoints,
                                 FeatureTrackDetector* feat_tracker_detector,
                                 std::vector<cv::KeyPoint>* key_pnts_ptr,
                                 cv::Mat_<uchar>* mask_with_of_out_ptr,
                                 cv::Mat* orb_feat_OF_ptr = nullptr,
                                 const cv::Vec2f& init_pixel_shift = cv::Vec2f(0, 0),
                                 const cv::Matx33f* K_ptr = nullptr,
                                 const cv::Mat_<float>* dist_ptr = nullptr,
                                 const cv::Matx33f* old_R_new_ptr = nullptr,
                                 const bool absolute_static = false);

// [NOTE] This is pyramid interface is used by FeatureTrackDetctor
void propagate_with_optical_flow(const std::vector<cv::Mat>& img_in_smooth_pyramids,
                                 const cv::Mat_<uchar>& mask,
                                 const std::vector<cv::Mat>& pre_image_pyramids,
                                 const cv::Mat& pre_image_orb_feature,
                                 const std::vector<cv::KeyPoint>& pre_image_keypoints,
                                 FeatureTrackDetector* feat_track_detector,
                                 std::vector<cv::KeyPoint>* key_pnts_ptr,
                                 cv::Mat_<uchar>* mask_with_of_out_ptr,
                                 cv::Mat* orb_feat_OF_ptr,
                                 const cv::Vec2f& init_pixel_shift,
                                 const cv::Matx33f* K_ptr,
                                 const cv::Mat_<float>* dist_ptr,
                                 const cv::Matx33f* old_R_new_ptr,
                                 const bool absolute_static);

inline cv::Mat fast_pyra_down_original(const cv::Mat& img_in_smooth) {
  constexpr int compress_ratio = 2;
  cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                       img_in_smooth.cols / compress_ratio,
                       CV_8U);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(img_in_smooth.type(), CV_8U);
#endif
  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  const int width_step_small = img_in_small.step1();
  for (int y = 0; y < img_in_small.rows; y++) {
    for (int x = 0; x < img_in_small.cols; x++) {
      // do not use .at<char> which is slow
      const int shift0 = (y * compress_ratio) * width_step_in + x * compress_ratio;
      const int shift1 = shift0 + width_step_in;
      int sum =
          static_cast<int>(*(img_in_smooth.data + shift0)) +
          static_cast<int>(*(img_in_smooth.data + shift1)) +
          static_cast<int>(*(img_in_smooth.data + shift0 + 1)) +
          static_cast<int>(*(img_in_smooth.data + shift1 + 1));
      *(img_in_small.data + y * width_step_small + x) = static_cast<uchar>(sum / 4);
    }
  }
  return img_in_small;
}

// make sure 0x00 keeps 0x00 in the next level
inline cv::Mat fast_mask_pyra_down(const cv::Mat& mask) {
  constexpr int compress_ratio = 2;
  cv::Mat mask_small(mask.rows / compress_ratio,
                     mask.cols / compress_ratio,
                     CV_8U);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(mask.type(), CV_8U);
#endif
  // use our own pyra down for faster performance
  const int width_step_in = mask.step1();
  const int width_step_small = mask_small.step1();
  for (int y = 0; y < mask_small.rows; y++) {
    for (int x = 0; x < mask_small.cols; x++) {
      // do not use .at<char> which is slow
      const int shift0 = (y * compress_ratio) * width_step_in + x * compress_ratio;
      const int shift1 = shift0 + width_step_in;
      if (*(mask.data + shift0) == 0x00 ||
          *(mask.data + shift1) == 0x00 ||
          *(mask.data + shift0 + 1) == 0x00 ||
          *(mask.data + shift1 + 1) == 0x00) {
        *(mask_small.data + y * width_step_small + x) = 0x00;
      } else {
        *(mask_small.data + y * width_step_small + x) = 0xff;
      }
    }
  }
  return mask_small;
}

}  // namespace XP
#endif  // XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
