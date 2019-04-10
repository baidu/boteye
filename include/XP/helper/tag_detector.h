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
#ifndef XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_
#define XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_

// april tag
#include <apriltag.h>
#include <xp_utilities.h>
#include <opencv2/core/types.hpp>  // For cv::Keypoint
#include <string>
#include <vector>

namespace XP {
class AprilTagDetector {
 public:
  explicit AprilTagDetector(const std::string& tag_family = "36h11");
  ~AprilTagDetector();
  int detect(const cv::Mat& img_in_raw,
             std::vector<cv::KeyPoint>* key_pnts_ptr,
             const float min_edge_pix = 0.f,
             const bool mirror = false);  // true if the tag is mirrored.
  // Refine decode can be slow.  Use with caution.
  // [NOTE] In general, 36h11 decoding is more robust than 25h7
  bool set_refine_decode(const bool refine_decode);

 private:
  std::string tag_family_;
  apriltag_family_t * tf_;
  apriltag_detector_t* td_;
};

// [NOTE] Assume the tag is cropped from the huge concatenated tag 36h11 image with
// tag ids ranging from 0 to 586 and specific layout below:
// 00 01 02 03 ... 23
// 24 25 26 27 ... 47
// 48 49 50 51 ... 71
// 72 73 74 75 ... 95
// 96 97 98 99 ... 111
// ...
// Each tag contains 4 corners, whose corner_id is,
// 1  2
// 3  4
// The resulting unique det_id for each corner = tag_id * 1 + corner_id
// [NOTE] Our tag detector return *negative* trace_ids.  We need to flip it here
// before further conversion to tag_id.
cv::Point3f id2coordinate_36h11(const int trace_id,
                                const float square_size,
                                const float gap_square_ratio);
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_
