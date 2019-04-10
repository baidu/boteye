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
#ifndef XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_

#include <opencv2/core.hpp>
#include <vector>

namespace XP {
class AutoWhiteBalance {
 public:
  inline AutoWhiteBalance(bool use_preset = false, float coeff_r = 1.f,
    float coeff_g = 1.f, float coeff_b = 1.f) :
    m_use_preset_(use_preset) {
    m_coeff_r_ = coeff_r;
    m_coeff_g_ = coeff_g;
    m_coeff_b_ = coeff_b;
  }
  inline ~AutoWhiteBalance() {
  }
  inline void run(cv::Mat* rgb_img_ptr) {
    CHECK_EQ(rgb_img_ptr != NULL, true);
    CHECK_EQ(rgb_img_ptr->channels(), 3);
    CHECK_EQ(rgb_img_ptr->type(), CV_8UC3);
    if (!m_use_preset_) {
      compute_AWB_coefficients(*rgb_img_ptr);
    }
    correct_white_balance_coefficients(rgb_img_ptr);
  }

  inline void setWhiteBalancePresetMode(float coeff_r,
    float coeff_g, float coeff_b) {
    m_coeff_r_ = coeff_r;
    m_coeff_g_ = coeff_g;
    m_coeff_b_ = coeff_b;
    m_use_preset_ = true;
  }

  inline void setManualWhiteBalanceMode() {
    m_use_preset_ = true;
  }

  inline void setAutoWhiteBalanceMode() {
    m_use_preset_ = false;
  }
  void compute_AWB_coefficients(const cv::Mat& rgb_img_);
  void correct_white_balance_coefficients(cv::Mat* rgb_img_ptr);
  const float r_coeff() const {
    return m_coeff_r_;
  }
  const float g_coeff() const {
    return m_coeff_g_;
  }
  const float b_coeff() const {
    return m_coeff_b_;
  }

  float& r_coeff() {
    return m_coeff_r_;
  }
  float& g_coeff() {
    return m_coeff_g_;
  }
  float& b_coeff() {
    return m_coeff_b_;
  }
 private:
  void compute_RGB_mean(const cv::Mat& rgb_img_,
                               uint32_t* ptr_r_mean,
                               uint32_t* ptr_g_mean,
                               uint32_t* ptr_b_mean) const;
#ifdef __ARM_NEON__
  void compute_RGB_mean_neon(const cv::Mat& rgb_img_,
                                    uint32_t* ptr_r_mean,
                                    uint32_t* ptr_g_mean,
                                    uint32_t* ptr_b_mean) const;
  void correct_white_balance_coefficients_neon(cv::Mat* rgb_img_ptr);
#endif  // __ARM_NEON__

  bool m_use_preset_;
  float m_coeff_r_;
  float m_coeff_g_;
  float m_coeff_b_;
};
// return true if better value if found
bool computeNewGainAndExposure(const cv::Mat& raw_img,
                               uint32_t* gain_ptr,
                               uint32_t* exposure_ptr,
                               const float max_bright_pixel_ratio = 0.6,
                               const float min_bright_pixel_ratio = 0.4,
                               const int target_brightness = 100);
// return true if new aec_index is found
bool computeNewAecTableIndex(const cv::Mat& raw_img,
                             int* aec_index_ptr);

int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr = nullptr);

float matchingHistogram(const std::vector<int>& hist_src,
                        const std::vector<int>& hist_tgt,
                        const float init_scale);

void gridBrightDarkAdjustBrightness(const cv::Mat& raw_img,
                                    int* adjusted_pixel_val_ptr);
int drawHistogram(cv::Mat* img_hist,
                  const std::vector<int>& histogram,
                  bool auto_scale = false);

// img_raw .* flat_field_coef = img_corrected
// coef(i, j) = (1 + (i - cy) * (i - cy) + (j - cx) * (j - cx)) ^ 2 / radius ^ 4
bool computeFlatFieldCoef(const cv::Size& img_size,
                          cv::Mat_<float>* flat_field_coef,
                          float radius = 380);  // for 480 x 752 img
bool applyFlatFieldCoef(const uchar* raw_img_data,  // e.g. pFrameData->leftData,
                        const cv::Mat_<float>& flat_field_coef,  // inplace is ok
                        cv::Mat* dst_img);  // this has to be pre-allocated
}  // namespace XP

#endif  // XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_
