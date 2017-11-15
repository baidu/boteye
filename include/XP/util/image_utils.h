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
#ifndef XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_IMAGE_UTILS_H_

#include <opencv2/core.hpp>
#include <vector>

namespace XP {
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
void drawHistogram(cv::Mat* img_hist, const std::vector<int>& histogram);

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
