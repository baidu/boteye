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
#ifndef XP_INCLUDE_XP_UTIL_CALIBRATION_UTILS_H_
#define XP_INCLUDE_XP_UTIL_CALIBRATION_UTILS_H_
#include <opencv2/core.hpp>
#include <vector>
namespace XP {
bool check_grid_point_density(const std::vector<std::vector<cv::Point2f> >& detected_corners,
                              const cv::Size& img_size,
                              const float min_ratio,
                              const int valid_radius,
                              const cv::Point2f& pinhole,
                              const bool verbose = false,
                              cv::Mat* visualize_img = nullptr);  // mark the radius and squares
}  // namespace XP
#endif  // XP_INCLUDE_XP_UTIL_CALIBRATION_UTILS_H_
