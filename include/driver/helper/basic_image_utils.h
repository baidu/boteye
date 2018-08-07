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
#ifndef INCLUDE_DRIVER_HELPER_BASIC_IMAGE_UTILS_H_
#define INCLUDE_DRIVER_HELPER_BASIC_IMAGE_UTILS_H_

#include <driver/helper/xp_logging.h>
#include <opencv2/core.hpp>
#include <vector>
#include <string>

namespace XPDRIVER {
class AutoWhiteBalance {
 public:
  inline AutoWhiteBalance(bool use_preset = false, float coeff_r = 1.f,
    float coeff_g = 1.f, float coeff_b = 1.f) :
    m_use_preset_(use_preset) {
    if (m_use_preset_) {
      m_coeff_r_ = coeff_r;
      m_coeff_g_ = coeff_g;
      m_coeff_b_ = coeff_b;
    }
  }
  inline ~AutoWhiteBalance() {
  }
  inline void run(cv::Mat* rgb_img_ptr) {
    XP_CHECK_EQ(rgb_img_ptr != NULL, true);
    XP_CHECK_EQ(rgb_img_ptr->channels(), 3);
    XP_CHECK_EQ(rgb_img_ptr->type(), CV_8UC3);
    if (!m_use_preset_) {
      compute_AWB_coefficients(*rgb_img_ptr);
    }
#ifdef __ARM_NEON__
    correct_white_balance_coefficients_neon(rgb_img_ptr);
#else
    correct_white_balance_coefficients(rgb_img_ptr);
#endif  // __ARM_NEON__
  }

  inline void run_original(cv::Mat* rgb_img_ptr) {
    XP_CHECK_EQ(rgb_img_ptr != NULL, true);
    XP_CHECK_EQ(rgb_img_ptr->channels(), 3);
    XP_CHECK_EQ(rgb_img_ptr->type(), CV_8UC3);
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

  inline void setAutoWhiteBalanceMode() {
    m_use_preset_ = false;
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

  void compute_AWB_coefficients(const cv::Mat& rgb_img_);
  void correct_white_balance_coefficients(cv::Mat* rgb_img_ptr);

  bool m_use_preset_;
  float m_coeff_r_;
  float m_coeff_g_;
  float m_coeff_b_;
};

bool computeNewAecTableIndex(const cv::Mat& raw_img,
                             const bool smooth_aec,
                             const uint32_t AEC_steps,
                             int* aec_index_ptr);

int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr = nullptr);

void gridBrightDarkAdjustBrightness(const cv::Mat& raw_img,
                                    int* adjusted_pixel_val_ptr);
}  // namespace XPDRIVER

#endif  // INCLUDE_DRIVER_HELPER_BASIC_IMAGE_UTILS_H_
