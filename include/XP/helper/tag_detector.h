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
#ifndef XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_
#define XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_

// april tag
#include <apriltag.h>
#include <xp_utilities.h>
#include <opencv2/core/types.hpp>  // For cv::Keypoint
#include <vector>

namespace XP {
class AprilTagDetector {
 public:
  AprilTagDetector();
  ~AprilTagDetector();
  int detect(const cv::Mat& img_in_raw,
             std::vector<cv::KeyPoint>* key_pnts_ptr,
             cv::Mat* orb_feat_ptr = nullptr);  // encode april tag as orb
 private:
  apriltag_family_t * tf_;
  apriltag_detector_t* td_;
};

}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_TAG_DETECTOR_H_
