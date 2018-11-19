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

#ifndef APRILTAG_XP_UTILITIES_H  // NOLINT
#define APRILTAG_XP_UTILITIES_H

#include <image_u8.h>
#include <pnm.h>
#include <apriltag.h>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <vector>

namespace MARKER {

typedef struct apriltag_result {
  cv::Point2f location;
  int id;
} at_res;

cv::Matx31d generate_C_om_O_from_markers(
    const cv::Mat& in_mat,
    const cv::Matx<double, 5, 1>& dist_coeffs,
    const cv::Matx33d& camera_matrix,
    const int width,
    const int height);

std::vector<cv::Point3f> calc_corner_positions_from_opencv_mat(
    const cv::Mat& mat,
    const int width,
    const int height);

void generate_tag_results_from_opencv_mat(
    const cv::Mat& mat,
    std::vector<cv::Point2f>* corner_positions,
    std::vector<int>* corner_ids);
apriltag_detector_t* create_apriltag_detector_t();
int det_tag(
    apriltag_detector_t* td_ptr,
    const cv::Mat& mat,
    std::vector<cv::Point2f>* corner_positions,
    std::vector<int>* corner_ids,
    const float min_edge_pix);

};  // namespace MARKER

#endif  // APRILTAG_XP_UTILITIES_H  // NOLINT
