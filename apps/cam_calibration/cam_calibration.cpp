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
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <XP/helper/tag_detector.h>  // For april tag detector
#include <XP/helper/param.h>  // For load / save calib param
#include <XP/util/calibration_utils.h>
#include <Eigen/LU>  // For Matrix4f inverse
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <set>
#include <chrono>
#include <limits>

using std::cout;
using std::endl;
using std::vector;
using namespace boost::filesystem;  // NOLINT
DEFINE_string(sensor_type,
              "UNKNOWN", "XP or XP2 or XP3 or FACE XPIRL or XPIRL2 or XPIRL3 or UDP (receiving)");
DEFINE_string(device_id, "", "device id of duo camera");
DEFINE_string(record_path, "", "folder containing l and r folders");
DEFINE_string(img_suffix, "png", "image suffix");
DEFINE_bool(show_reproj, false, "whether or not show image with points in a debug window");
DEFINE_bool(show_det, false, "whether or not show image with tag detection");
DEFINE_bool(show_coverage, false, "whether or not show point coverage w/o running calibration");
DEFINE_bool(fish_eye_model, false, "whether or not use the fish eye model");
DEFINE_int32(valid_radius, 360,
             "The radius in pixel to check the point coverage from the pinhole center. "
             "Suggested value: 360 for 120 deg FOV and 220 for 170 deg FOV.");
DEFINE_bool(no_distribution_check, false, "disable point distribution check");
DEFINE_double(min_ratio, 0.4,
              "The required minimum ratio of detected points over the average per grid");
DEFINE_double(square_size, -1, "length of the side of one tag");
DEFINE_double(gap_square_ratio, 0.25, "length of the gap between tags");  // april tag pattern
DEFINE_string(load_calib, "", "calib file to load (old cv format)");
DEFINE_string(load_calib_yaml, "", "calib file to load (new yaml format)");
DEFINE_string(save_calib, "", "calib file to save (old cv format)");
DEFINE_string(save_calib_yaml, "", "calib file to save (new yaml format)");
DEFINE_bool(verify_mode, false, "verify reproj error of existing calib file");
DEFINE_bool(no_ransac, false, "no ransac");
DEFINE_bool(extrinsics_only, false, "only perform extrinsics calibration");
DEFINE_int32(display_timeout, 500, "how many millisec does a window display. 0 is inf");
DEFINE_int32(dist_order, 8, "8 (k1 k2 0 0 k3 k4 k5 k6) or 6 (k1 k2 0 0 k3) or 4 (k1 k2 p1 p2)"
             " or 2 (k1 k2)");
DEFINE_bool(single_cam_mode, false, "Only calibrate one camera.");

// [NOTE] Assume 5x7 april tag is used.
const int kFullCorners = 5 * 7 * 4;
cv::Point3f id2coordinate(const int det_id) {
  // compute 3d position
  // tag corner ids are
  // 1 2
  // 4 3
  // board is 5x7
  // Arrangement of tag ids is
  // 00 01 02 03 04 05 06
  // 24 25 26 27 28 29 30
  // 48 49 50...
  // 72 ..
  // 96 ..             102
  cv::Point3f coordinate;
  const int tag_id = det_id / 10;
  const int col_id = tag_id % 24;
  const int row_id = tag_id / 24;
  CHECK_LT(col_id, 7) << "tag_id " << tag_id << " does not belong to the tag board";
  const int corner_id = det_id % 10;
  // std::cout << tag_id << ":" << corner_id << " -> " << corner_positions[i] << std::endl;
  const float x_upper_left =
      col_id * (FLAGS_square_size + FLAGS_square_size * FLAGS_gap_square_ratio);
  const float y_upper_left =
      row_id * (FLAGS_square_size + FLAGS_square_size * FLAGS_gap_square_ratio);
  coordinate.x = x_upper_left;
  coordinate.y = y_upper_left;
  coordinate.z = 0;
  CHECK_GE(corner_id, 1);
  CHECK_LE(corner_id, 4);
  if (corner_id == 2 || corner_id == 3) {
    coordinate.x += FLAGS_square_size;
  }
  if (corner_id == 3 || corner_id == 4) {
    coordinate.y += FLAGS_square_size;
  }
  return coordinate;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = 1;
  if (FLAGS_record_path.size() == 0) {
    google::ShowUsageWithFlags(argv[0]);
    return -1;
  }
  if (FLAGS_square_size < 0 && !FLAGS_show_coverage) {
    LOG(ERROR) << "You need to set square_size";
    google::ShowUsageWithFlags(argv[0]);
    return -1;
  }
  if (FLAGS_dist_order != 8 &&
      FLAGS_dist_order != 6 &&
      FLAGS_dist_order != 4 &&
      FLAGS_dist_order != 2) {
    LOG(ERROR) << "distortion order has to be 8 or 6 or 4 or 2";
    return -1;
  }
  if (FLAGS_extrinsics_only && (FLAGS_load_calib_yaml.empty())) {
    LOG(ERROR) << "Set load_calib_yaml to use extrinsics_only";
    return -1;
  }
  int camera_max_index = 2;
  if (FLAGS_single_cam_mode) {
    camera_max_index = 1;
  }

  // init april tag detector
  XP::AprilTagDetector april_tag_detector;
  vector<std::string> lr_folders(2);
  lr_folders[0] = "l";
  lr_folders[1] = "r";
  XP::DuoCalibParam duo_calib_param;
  typedef vector<cv::Point2f> DetectedCorners;
  typedef vector<cv::Point3f> ObjectCorners;
  vector<vector<DetectedCorners>> detected_corners_all_lr(2);
  // some times an img in left may not have corners but the right image has
  // so detected_corners_all_lr[0] and detected_corners_all_lr[1] have
  // different number of elements.
  // We record the img ids for stereo calibration
  vector<vector<int>> detected_corners_all_img_id_lr(2);
  vector<vector<ObjectCorners>> corresponding_board_coordinates_all_lr(2);
  vector<vector<vector<int>>> corner_ids_all_lr(2);
  vector<std::set<int>> inlier_ids_lr(2);
  vector<cv::Mat> res_debug_imgs(2);
  vector<cv::Mat> fisheye_res_debug_imgs(2);
  cv::Size& img_size = duo_calib_param.Camera.img_size;
  img_size = cv::Size(0, 0);
  std::stringstream ss_log;

  // Detect corners first
  for (int lr = 0; lr < camera_max_index; ++lr) {
    boost::filesystem::path folder_path(FLAGS_record_path + "/" + lr_folders[lr]);
    if (!is_directory(folder_path)) {
      LOG(ERROR) << folder_path << " is not a folder";
      return -1;
    }
    vector<DetectedCorners>& detected_corners_all = detected_corners_all_lr[lr];
    vector<int>& detected_corners_all_img_id = detected_corners_all_img_id_lr[lr];
    vector<ObjectCorners>& corresponding_board_coordinates_all =
        corresponding_board_coordinates_all_lr[lr];
    detected_corners_all.reserve(200);
    detected_corners_all_img_id.reserve(200);
    corresponding_board_coordinates_all.reserve(200);
    corner_ids_all_lr[lr].reserve(200);
    const cv::TermCriteria pixel_refinement_criteria
        = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    int img_counter = 0;
    vector<path> file_names_vec;
    copy(directory_iterator(folder_path), directory_iterator(), std::back_inserter(file_names_vec));
    std::sort(file_names_vec.begin(), file_names_vec.end());
    for (auto it_path : file_names_vec) {
      if (it_path.extension().string() == "." + FLAGS_img_suffix) {
        cv::Mat img_gray_original = cv::imread(it_path.string(), CV_LOAD_IMAGE_GRAYSCALE);
        // cv::Mat img_gray_original;
        // cv::cvtColor(img_original, img_gray_original, CV_BGR2GRAY);
        cv::Mat img_gray = img_gray_original;
        if (img_gray.rows == 0) {
          LOG(ERROR) << it_path << " cannot be read by opencv";
        }
        if (img_size.width == 0) {
          img_size = img_gray.size();
        }
        // Sanity check. Make sure all images are of the same size.
        CHECK_EQ(img_size.width, img_gray.cols);
        CHECK_EQ(img_size.height, img_gray.rows);
        std::vector<cv::KeyPoint> kps;
        int det_num = april_tag_detector.detect(img_gray, &kps);
        std::vector<cv::Point2f> corner_positions(kps.size());
        std::vector<int> corner_ids(kps.size());
        for (size_t i = 0; i < kps.size(); ++i) {
          corner_positions[i] = kps[i].pt;
          corner_ids[i] = kps[i].class_id;
        }
        corner_ids_all_lr[lr].push_back(corner_ids);
        /*
        std::cout << "detected " << det_num << " tags from "
                  << it_path.filename() << " size " << img_size << std::endl;
        */
        if (det_num == 0) {
          ++img_counter;
          continue;
        }
        detected_corners_all_img_id.push_back(img_counter);
        ++img_counter;
        // vector<cv::Point2f> detected_corners(det_num * 4);
        vector<cv::Point3f> corresponding_board_coordinates(det_num * 4);
        CHECK_EQ(det_num * 4, corner_ids.size());
        for (size_t i = 0; i < corner_ids.size(); i++) {
          corresponding_board_coordinates[i] = id2coordinate(corner_ids[i]);
          // shift detection result
          corner_positions[i].x -= 0.5;
          corner_positions[i].y -= 0.5;
        }
        const auto corner_pos_before_refinement = corner_positions;
        cv::cornerSubPix(img_gray, corner_positions,
                         cv::Size(2, 2),  // Half of the side length of the search window
                         cv::Size(-1, -1),  // zeroZone
                         pixel_refinement_criteria);
        detected_corners_all.push_back(corner_positions);
        corresponding_board_coordinates_all.push_back(corresponding_board_coordinates);
        if (FLAGS_show_det) {
          cv::Mat detection_img;
          cv::cvtColor(img_gray, detection_img, CV_GRAY2RGB);
          for (size_t it_corner = 0; it_corner < corner_positions.size(); ++it_corner) {
            cv::circle(detection_img, corner_positions[it_corner], 2, cv::Scalar(0, 0xff, 0));
            cv::line(detection_img, corner_positions[it_corner],
                     corner_pos_before_refinement[it_corner], cv::Scalar(0, 0, 0xff));
          }
          cv::imshow("detection", detection_img);
          cv::waitKey(FLAGS_display_timeout);
        }
      }
    }
    const size_t img_num_all = detected_corners_all.size();
    CHECK_EQ(img_num_all, corresponding_board_coordinates_all.size());
    cout << "Detected tags from " << img_num_all << " imgs from Camera " << lr << endl;
    ss_log << "Camera " << lr << ": detected tags from " << img_num_all << " imgs" << endl;
  }

  // Check point coverage
  bool point_coverage_ok = true;
  if (!FLAGS_no_distribution_check) {
    cv::Mat det_img(img_size.height, img_size.width * camera_max_index, CV_8UC3);
    det_img.setTo(cv::Vec3b(0, 0, 0));
    for (int cam_idx = 0; cam_idx < camera_max_index; ++cam_idx) {
      cv::Mat det_img_this = det_img(cv::Rect(img_size.width * cam_idx, 0,
                                              img_size.width, img_size.height));
      const vector<DetectedCorners>& detected_corners_all = detected_corners_all_lr[cam_idx];
      float min_ratio = std::min(FLAGS_min_ratio, 0.9);
      if (XP::check_grid_point_density(detected_corners_all,
                                       img_size,
                                       min_ratio,
                                       FLAGS_valid_radius,
                                       cv::Point2f(img_size.width / 2, img_size.height / 2),
                                       true /*verbose*/,
                                       &det_img_this)) {
        std::cout << "[OK] point distribution check for camera " << cam_idx << "\n";
      } else {
        std::cout << "WARNING: suggest to take more pictures for camera " << cam_idx << "!!!\n";
        point_coverage_ok = false;
      }
      size_t corner_count = 0;
      const size_t img_num_all = detected_corners_all.size();
      for (size_t img_it = 0; img_it < img_num_all; ++img_it) {
        const auto& corners = detected_corners_all[img_it];
        corner_count += corners.size();
        for (const cv::Point2f& corner : corners) {
          cv::circle(det_img_this, corner, 1, cv::Scalar(0xff, 0xff, 0));
        }
      }
      std::cout << "Camera " << cam_idx << ": total " << img_num_all << " images with "
                << corner_count << " corners detected\n";
    }
    std::string det_filename = "/tmp/calib_coverage.png";
    if (!FLAGS_save_calib_yaml.empty()) {
      boost::filesystem::path calib_path(FLAGS_save_calib);
      det_filename = calib_path.parent_path().string() + "/calib_coverage.png";
    } else if (!FLAGS_save_calib.empty()) {
      boost::filesystem::path calib_path(FLAGS_save_calib_yaml);
      det_filename = calib_path.parent_path().string() + "/calib_coverage.png";
    }
    cv::imwrite(det_filename, det_img);
    if (FLAGS_show_coverage) {
      cv::imshow("Camera coverage", det_img);
      cv::waitKey(0);
      return 0;
    }
  }
  if (!point_coverage_ok) {
    return -1;
  }

  // Do intrinsics calibration
  for (int lr = 0; lr < camera_max_index; ++lr) {
    vector<DetectedCorners>& detected_corners_all = detected_corners_all_lr[lr];
    vector<ObjectCorners>& corresponding_board_coordinates_all =
        corresponding_board_coordinates_all_lr[lr];
    std::set<int>& inlier_ids = inlier_ids_lr[lr];
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist = cv::Mat::zeros(8, 1, CV_64F);
    float best_reproj_error = std::numeric_limits<float>::max();
    vector<DetectedCorners> detected_corners_inliers;
    vector<ObjectCorners> corresponding_board_coordinates_inliers;
    if (!FLAGS_verify_mode && !FLAGS_extrinsics_only) {
      // run normal model to init
      // use ransac to calib
      vector<size_t> exp_ids(detected_corners_all.size());
      for (size_t i = 0; i < detected_corners_all.size(); ++i) {
        exp_ids[i] = i;
      }
      int exp_num = 30;
      float ransac_reproj2_thresh = 2 * 2;
      int ransac_use_img_num = 10;
      const int min_inlier_num = detected_corners_all.size() / 3 * 2;
      if (FLAGS_no_ransac) {
        // use all imgs
        ransac_use_img_num = detected_corners_all.size();
        exp_num = 1;
        ransac_reproj2_thresh = std::numeric_limits<float>::max();
      }
      if (detected_corners_all.size() < ransac_use_img_num) {
        LOG(ERROR) << "Only " << detected_corners_all.size() << " images contain tag detections";
        return -1;
      }
      for (int it_exp = 0; it_exp < exp_num; it_exp++) {
        std::random_shuffle(exp_ids.begin(), exp_ids.end());
        vector<cv::Mat> rvecs, tvecs;
        vector<vector<cv::Point2f> > detected_corners_ransac(ransac_use_img_num);
        vector<vector<cv::Point3f> > corresponding_board_coordinates_ransac(ransac_use_img_num);
        for (int i = 0; i < ransac_use_img_num; ++i) {
          detected_corners_ransac[i] = detected_corners_all[exp_ids[i]];
          corresponding_board_coordinates_ransac[i] =
              corresponding_board_coordinates_all[exp_ids[i]];
        }
        cv::Mat K_ransac = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_ransac;
        int calib_flag;
        if (FLAGS_dist_order == 2) {
          calib_flag = (CV_CALIB_ZERO_TANGENT_DIST |
                        CV_CALIB_FIX_K3 |
                        CV_CALIB_FIX_K4 |
                        CV_CALIB_FIX_K5 |
                        CV_CALIB_FIX_K6);
        } else if (FLAGS_dist_order == 4) {
          calib_flag = (CV_CALIB_FIX_K3 |
                        CV_CALIB_FIX_K4 |
                        CV_CALIB_FIX_K5 |
                        CV_CALIB_FIX_K6);
        } else if (FLAGS_dist_order == 6) {
          calib_flag = CV_CALIB_ZERO_TANGENT_DIST;
        } else if (FLAGS_dist_order == 8) {
          calib_flag = (CV_CALIB_ZERO_TANGENT_DIST |
                        CV_CALIB_RATIONAL_MODEL);
        } else {
          LOG(FATAL) << "FLAGS_dist_order = " << FLAGS_dist_order;
        }
        const float seed_reproj_error = cv::calibrateCamera(
          corresponding_board_coordinates_ransac, detected_corners_ransac,
          img_size, K_ransac, dist_ransac, rvecs, tvecs, calib_flag);
        if (dist_ransac.rows > 0) {
          dist_ransac = dist_ransac.t();
        }
        if (seed_reproj_error * seed_reproj_error > ransac_reproj2_thresh) {
          // too big reproj error
          cout << "ransac exp " << it_exp << " seed_reproj_error " << seed_reproj_error
               << " too big ransac_reproj2_thresh " << ransac_reproj2_thresh << std::endl;
          continue;
        }
        // recompute reproj from all imgs
        std::set<int> good_ids;
        vector<DetectedCorners> reproj_corners_this_exp;
        reproj_corners_this_exp.reserve(corresponding_board_coordinates_all.size());
        for (size_t it_img = 0; it_img < corresponding_board_coordinates_all.size(); it_img++) {
          float reproj_error2_this_img = 0;
          int reproj_count_this_img = 0;
          vector<cv::Point2f> corner_reproj;
          cv::Mat rvec, tvec;
          cv::solvePnP(corresponding_board_coordinates_all[it_img],
                       detected_corners_all[it_img],
                       K_ransac, dist_ransac, rvec, tvec);
          cv::projectPoints(corresponding_board_coordinates_all[it_img],
                            rvec,
                            tvec,
                            K_ransac, dist_ransac,
                            corner_reproj);
          CHECK_EQ(corner_reproj.size(), detected_corners_all[it_img].size());
          for (size_t it_corner = 0; it_corner < corner_reproj.size(); it_corner++) {
            float x_diff = detected_corners_all[it_img][it_corner].x - corner_reproj[it_corner].x;
            float y_diff = detected_corners_all[it_img][it_corner].y - corner_reproj[it_corner].y;
            float diff = x_diff * x_diff + y_diff * y_diff;
            reproj_error2_this_img += diff;
            reproj_count_this_img++;
          }

          if (reproj_error2_this_img / reproj_count_this_img < ransac_reproj2_thresh) {
            good_ids.emplace(it_img);
            reproj_corners_this_exp.push_back(corner_reproj);
          }
        }
        if (!FLAGS_no_distribution_check) {
          if (!XP::check_grid_point_density(reproj_corners_this_exp,
                                            img_size,
                                            FLAGS_min_ratio,
                                            FLAGS_valid_radius,
                                            cv::Point2f(K_ransac.at<double>(0, 2),
                                                        K_ransac.at<double>(1, 2)),
                                            true /*verbose*/)) {
            cout << "ransac exp " << it_exp << " seed_reproj_error " << seed_reproj_error
                << " inlier pnts doesn't have good point coverage\n";
            continue;
          }
        }
        if (min_inlier_num > good_ids.size()) {
          cout << "ransac exp " << it_exp << " seed_reproj_error " << seed_reproj_error
               << " inlier num " << good_ids.size()
               << " < " << min_inlier_num << std::endl;
          continue;
        }
        vector<vector<cv::Point2f> > detected_corners_good;
        detected_corners_good.reserve(good_ids.size());
        vector<vector<cv::Point3f> > corresponding_board_coordinates_good;
        corresponding_board_coordinates_good.reserve(good_ids.size());
        for (const auto& id : good_ids) {
          detected_corners_good.push_back(detected_corners_all[id]);
          corresponding_board_coordinates_good.push_back(corresponding_board_coordinates_all[id]);
        }
        float final_reproj_error = seed_reproj_error;
        if (!FLAGS_no_ransac) {
          int calib_flag;
          if (FLAGS_dist_order == 2) {
            calib_flag = (CV_CALIB_USE_INTRINSIC_GUESS |
                          CV_CALIB_ZERO_TANGENT_DIST |
                          CV_CALIB_FIX_K3 |
                          CV_CALIB_FIX_K4 |
                          CV_CALIB_FIX_K5 |
                          CV_CALIB_FIX_K6);
          } else if (FLAGS_dist_order == 4) {
            calib_flag = (CV_CALIB_USE_INTRINSIC_GUESS |
                          CV_CALIB_FIX_K3 |
                          CV_CALIB_FIX_K4 |
                          CV_CALIB_FIX_K5 |
                          CV_CALIB_FIX_K6);
          } else if (FLAGS_dist_order == 6) {
            calib_flag = (CV_CALIB_USE_INTRINSIC_GUESS |
                          CV_CALIB_ZERO_TANGENT_DIST);
          } else if (FLAGS_dist_order == 8) {
            calib_flag = (CV_CALIB_USE_INTRINSIC_GUESS |
                          CV_CALIB_ZERO_TANGENT_DIST |
                          CV_CALIB_RATIONAL_MODEL);
          } else {
            LOG(FATAL) << "FLAGS_dist_order = " << FLAGS_dist_order;
          }
          final_reproj_error = cv::calibrateCamera(corresponding_board_coordinates_good,
                                                   detected_corners_good,
                                                   img_size, K_ransac, dist_ransac,
                                                   rvecs, tvecs, calib_flag);
        }
        if (best_reproj_error > final_reproj_error) {
          best_reproj_error = final_reproj_error;
          K = K_ransac.clone();
          dist = dist_ransac.clone();
          detected_corners_inliers = detected_corners_good;
          corresponding_board_coordinates_inliers = corresponding_board_coordinates_good;
          inlier_ids = good_ids;
          VLOG(1) << "new best K\n" << K << endl << "dist =\n" << dist << std::endl;
        }
        cout << "ransac exp " << it_exp << " seed_reproj_error " << seed_reproj_error
             << " final_reproj_error " << final_reproj_error
             << " inlier num " << corresponding_board_coordinates_good.size() << std::endl;
        if (corresponding_board_coordinates_good.size() == detected_corners_all.size()) {
          cout << "all imgs are inliers. Done" << std::endl;
          break;
        }
      }
      if (best_reproj_error > 4 && !FLAGS_no_ransac) {
        cout << "calib error too big. FAILED" << endl;
        return -1;
      }
      cout << "final K =\n" << K << std::endl;
      cout << "final dist =\n" << dist << std::endl;
      for (int i = 0; i < 3; ++i) {
        for  (int j = 0; j < 3; ++j) {
          duo_calib_param.Camera.cv_camK_lr[lr](i, j) = K.at<double>(i, j);
        }
      }
      for (int i = 0; i < 3; ++i) {
        for  (int j = 0; j < 3; ++j) {
          duo_calib_param.Camera.cameraK_lr[lr](i, j) = K.at<double>(i, j);
        }
      }
      duo_calib_param.Camera.cv_dist_coeff_lr[lr].create(dist.rows, 1);
      CHECK_EQ(dist.cols, 1);  // col vector
      for (int i = 0; i < dist.rows; ++i) {
        duo_calib_param.Camera.cv_dist_coeff_lr[lr](i) = dist.at<double>(i, 0);
      }
    } else {
      // verify_mode here
      if (FLAGS_load_calib.empty() && FLAGS_load_calib_yaml.empty()) {
        LOG(ERROR) << "load_calib and load_calib_yaml are both unset";
        return -1;
      }
      if (!FLAGS_load_calib.empty() && !FLAGS_load_calib_yaml.empty()) {
        LOG(ERROR) << "Only allow loading one type of calib file";
        return -1;
      }
      if (!FLAGS_load_calib.empty()) {
        CHECK(XP::load_camera_calib_param(FLAGS_load_calib, &duo_calib_param));
      }
      if (!FLAGS_load_calib_yaml.empty()) {
        CHECK(duo_calib_param.LoadCamCalibFromYaml(FLAGS_load_calib_yaml));
      }
      for (int i = 0; i < 3; ++i) {
        for  (int j = 0; j < 3; ++j) {
          K.at<double>(i, j) = duo_calib_param.Camera.cv_camK_lr[lr](i, j);
        }
      }
      for (int i = 0; i < duo_calib_param.Camera.cv_dist_coeff_lr[lr].rows; ++i) {
        dist.at<double>(i, 0) = duo_calib_param.Camera.cv_dist_coeff_lr[lr](i);
      }
      detected_corners_inliers = detected_corners_all;
      corresponding_board_coordinates_inliers = corresponding_board_coordinates_all;
      for (size_t i = 0; i < detected_corners_inliers.size(); ++i) {
        inlier_ids_lr[lr].emplace(i);
      }
    }
    // Compute and visualize error
    {
      cv::Mat& res_debug_img = res_debug_imgs[lr];
      res_debug_img.create(img_size, CV_8UC3);
      res_debug_img.setTo(cv::Vec3b(0, 0, 0));
      float reproj_error = 0;
      int reproj_count = 0;
      for (size_t it_img = 0; it_img < corresponding_board_coordinates_inliers.size(); it_img++) {
        float reproj_error_this_img = 0;
        float reproj_error_largest = 0;
        int reproj_count_this_img = 0;
        vector<cv::Point2f> corner_reproj;
        cv::Mat rvec, tvec;
        cv::solvePnP(corresponding_board_coordinates_inliers[it_img],
                     detected_corners_inliers[it_img], K, dist, rvec, tvec);
        VLOG(1) << "verify mode it_img " << it_img << " tvec=" << tvec.t() << endl;
        cv::projectPoints(corresponding_board_coordinates_inliers[it_img],
                          rvec,
                          tvec,
                          K, dist,
                          corner_reproj);
        CHECK_EQ(corner_reproj.size(), detected_corners_inliers[it_img].size());
        for (size_t it_corner = 0; it_corner < corner_reproj.size(); it_corner++) {
          float x_diff = detected_corners_inliers[it_img][it_corner].x - corner_reproj[it_corner].x;
          float y_diff = detected_corners_inliers[it_img][it_corner].y - corner_reproj[it_corner].y;
          float diff = x_diff * x_diff + y_diff * y_diff;
          float sqrt_diff = sqrt(diff);
          reproj_error_this_img += sqrt_diff;
          if (reproj_error_largest < sqrt_diff) {
            reproj_error_largest = sqrt_diff;
          }
          reproj_count_this_img++;
          cv::Scalar color(0, 0xff, 0);
          if (diff > 4) {
            color = cv::Scalar(0, 0, 0xff);
          } else if (diff > 1) {
            color = cv::Scalar(0, 0xff, 0xff);
          }
          cv::circle(res_debug_img, detected_corners_inliers[it_img][it_corner], 1, color);
          if (corner_reproj[it_corner].x >= 0 &&
              corner_reproj[it_corner].y >= 0 &&
              corner_reproj[it_corner].x < img_size.width - 1 &&
              corner_reproj[it_corner].y < img_size.height - 1) {
            cv::line(res_debug_img, detected_corners_inliers[it_img][it_corner],
                     corner_reproj[it_corner], color);
          }
        }
        reproj_count += reproj_count_this_img;
        reproj_error += reproj_error_this_img;
        VLOG(1) << "normal model img " << it_img << " avg reproj_error = "
                << reproj_error_this_img / reproj_count_this_img
                << " reproj_error largest " << reproj_error_largest
                << " corner# "<< reproj_count_this_img << std::endl;
      }
      cout << "normal model avg reproj_error = " << reproj_error / reproj_count
           << " corner# "<< reproj_count << std::endl;
      ss_log << "Camera " << lr << ": normal model avg reproj_error = "
             << reproj_error / reproj_count << " corner# "<< reproj_count << std::endl;

      if (FLAGS_show_reproj) {
        cv::imshow("radial dist model reproj " + boost::lexical_cast<std::string>(lr),
                   res_debug_img);
        // waitkey at the end
      }
    }
    // not used yet
    if (FLAGS_fish_eye_model) {
      // note opencv 3.0 assumes all board have the same # of corners.
      // This bug is fixed in https://github.com/opencv/opencv/pull/7026
      const int img_num = corresponding_board_coordinates_inliers.size();
      cout << "Calibrate fisheye model from " << img_num << " imgs" << endl;
      cv::Matx33d K_fe(K);
      // K33f(0, 0) = 500;
      // K33f(1, 1) = 500;
      cout << "K_fish_eye_init = " << K_fe << endl;
      cv::Vec4d D;
      for (int i = 0; i < 4; ++i) {
        D(i) = 0;  // = cv::Vec4d::zeros(); does not work
      }
      for (int i = 0; i < img_num; ++i) {
        const int corner_num = corresponding_board_coordinates_inliers[i].size();
        CHECK_EQ(corner_num, detected_corners_inliers[i].size());
        CHECK_GT(corner_num, 0);
      }
      std::vector<cv::Vec3d> rvecs(img_num);
      std::vector<cv::Vec3d> tvecs(img_num);
      // The returned value of cv::fisheye::calibrate is unknown
      CHECK_EQ(img_num, detected_corners_inliers.size());
      CHECK_GT(img_num, 0);
      double fisheye_calib_error = cv::fisheye::calibrate(
        corresponding_board_coordinates_inliers, detected_corners_inliers,
        img_size, K_fe, D, rvecs, tvecs,
        cv::fisheye::CALIB_USE_INTRINSIC_GUESS |
        cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC |
        cv::fisheye::CALIB_FIX_SKEW |
        // cv::fisheye::CALIB_FIX_K3 |
        // cv::fisheye::CALIB_FIX_K4 |
        cv::fisheye::CALIB_CHECK_COND,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-10));
      CHECK_EQ(rvecs.size(), corresponding_board_coordinates_inliers.size());
      CHECK_EQ(tvecs.size(), corresponding_board_coordinates_inliers.size());
      cout << "K_fish_eye = " << K_fe << std::endl;
      cout << "D = " << D << std::endl;

      cv::Mat& fisheye_res_debug_img = fisheye_res_debug_imgs[lr];
      fisheye_res_debug_img.create(img_size, CV_8UC3);
      fisheye_res_debug_img.setTo(cv::Vec3b(0, 0, 0));
      {
        float fish_eye_reproj_error = 0;
        int fish_eye_reproj_count = 0;
        for (size_t it_img = 0; it_img < corresponding_board_coordinates_inliers.size(); it_img++) {
          if (!corresponding_board_coordinates_inliers[it_img].empty()) {
            vector<cv::Point2f> corner_reproj;
            cv::fisheye::projectPoints(corresponding_board_coordinates_inliers[it_img],
                                       corner_reproj,
                                       rvecs[it_img],
                                       tvecs[it_img],
                                       K_fe, D);
            CHECK_EQ(corner_reproj.size(), detected_corners_inliers[it_img].size());
            for (size_t it_corner = 0; it_corner < corner_reproj.size(); it_corner++) {
              float x_diff =
              detected_corners_inliers[it_img][it_corner].x - corner_reproj[it_corner].x;
              float y_diff =
              detected_corners_inliers[it_img][it_corner].y - corner_reproj[it_corner].y;
              float diff = x_diff * x_diff + y_diff * y_diff;
              fish_eye_reproj_count++;
              fish_eye_reproj_error += sqrt(diff);
              cv::Scalar color(0, 0xff, 0);
              if (diff > 4) {
                color = cv::Scalar(0, 0, 0xff);
              } else if (diff > 1) {
                color = cv::Scalar(0, 0xff, 0xff);
              }
              cv::circle(fisheye_res_debug_img,
                         detected_corners_inliers[it_img][it_corner], 1, color);
              if (corner_reproj[it_corner].x >= 0 &&
                  corner_reproj[it_corner].y >= 0 &&
                  corner_reproj[it_corner].x < img_size.width - 1 &&
                  corner_reproj[it_corner].y < img_size.height - 1) {
                cv::line(fisheye_res_debug_img, detected_corners_inliers[it_img][it_corner],
                         corner_reproj[it_corner], color);
              }
            }
          }
        }
        cout << "fish eye avg reproj_error = " << fish_eye_reproj_error / fish_eye_reproj_count
             << " fisheye_calib_error = " << fisheye_calib_error
             << " corner# "<< fish_eye_reproj_count << std::endl;
        ss_log << "fish eye avg reproj_error = " << fish_eye_reproj_error / fish_eye_reproj_count
               << " fisheye_calib_error = " << fisheye_calib_error
               << " corner# "<< fish_eye_reproj_count << std::endl;
        if (FLAGS_show_reproj) {
          cv::imshow("fisheye model reproj", fisheye_res_debug_img);
        }
      }
    }
    if (FLAGS_show_reproj) {
      cv::waitKey(FLAGS_display_timeout);
    }
  }

  // calib stereo
  if (!FLAGS_single_cam_mode) {
    // the # of imgs of left and right may be different
    CHECK_GE(corner_ids_all_lr[0].size(), detected_corners_all_img_id_lr[0].size());
    CHECK_GE(corner_ids_all_lr[1].size(), detected_corners_all_img_id_lr[1].size());
    CHECK_EQ(detected_corners_all_lr[0].size(), detected_corners_all_img_id_lr[0].size());
    CHECK_EQ(detected_corners_all_lr[1].size(), detected_corners_all_img_id_lr[1].size());
    int it_left = 0, it_right = 0;
    vector<vector<DetectedCorners>> stereo_calib_2dcorners_lr(2);
    vector<ObjectCorners> stereo_calib_3d_coordinates;
    for (; it_left < detected_corners_all_lr[0].size()
           && it_right < detected_corners_all_lr[1].size(); ++it_left, ++it_right) {
      bool sync_flag = detected_corners_all_img_id_lr[0][it_left] ==
                       detected_corners_all_img_id_lr[1][it_right];
      while (!sync_flag) {
        if (detected_corners_all_img_id_lr[0][it_left] <
            detected_corners_all_img_id_lr[1][it_right]) {
          ++it_left;
        }
        if (detected_corners_all_img_id_lr[0][it_left] >
            detected_corners_all_img_id_lr[1][it_right]) {
          ++it_right;
        }
        if (it_left >= detected_corners_all_lr[0].size()) break;
        if (it_right >= detected_corners_all_lr[1].size()) break;
        if (detected_corners_all_img_id_lr[0][it_left] ==
            detected_corners_all_img_id_lr[1][it_right]) {
          sync_flag = true;
        }
      }
      if (!sync_flag) break;
      const int l_img_id = detected_corners_all_img_id_lr[0][it_left];
      const int r_img_id = detected_corners_all_img_id_lr[1][it_right];
      LOG(INFO) << "l " << it_left << " : " << l_img_id
                << " r " << it_right << " : " << r_img_id
                << " sync " << sync_flag;
      if (inlier_ids_lr[0].find(it_left) != inlier_ids_lr[0].end() &&
          inlier_ids_lr[1].find(it_right) != inlier_ids_lr[1].end()) {
        const vector<int>& corner_ids_l = corner_ids_all_lr[0][l_img_id];
        const vector<int>& corner_ids_r = corner_ids_all_lr[1][r_img_id];
        const DetectedCorners& det_corners_l = detected_corners_all_lr[0][it_left];
        const DetectedCorners& det_corners_r = detected_corners_all_lr[1][it_right];
        CHECK_EQ(corner_ids_l.size(), det_corners_l.size());
        CHECK_EQ(corner_ids_r.size(), det_corners_r.size());

        // Don't bother to check matching if either one view has too few corners
        if (corner_ids_l.size() < kFullCorners / 2 || corner_ids_r.size() < kFullCorners / 2) {
          continue;
        }
        DetectedCorners matched_corners_l;
        DetectedCorners matched_corners_r;
        ObjectCorners matched_3d_coords;
        for (int it_l = 0; it_l < corner_ids_l.size(); ++it_l) {
          int corner_id = corner_ids_l[it_l];
          bool found = false;
          int it_r = 0;
          for (; it_r < corner_ids_r.size(); ++it_r) {
            if (corner_ids_r[it_r] == corner_id) {
              found = true;
              break;
            }
          }
          if (found) {
            matched_corners_l.push_back(det_corners_l[it_l]);
            matched_corners_r.push_back(det_corners_r[it_r]);
            matched_3d_coords.push_back(id2coordinate(corner_id));
          }
        }
        CHECK_EQ(matched_corners_l.size(), matched_corners_r.size());
        CHECK_EQ(matched_corners_l.size(), matched_3d_coords.size());
        if (matched_3d_coords.size() > kFullCorners / 2) {
          stereo_calib_2dcorners_lr[0].push_back(matched_corners_l);
          stereo_calib_2dcorners_lr[1].push_back(matched_corners_r);
          stereo_calib_3d_coordinates.push_back(matched_3d_coords);
          LOG(INFO) << "Add to stereo calib matched corner size = " << matched_3d_coords.size();
        }
      }
    }
    CHECK_EQ(stereo_calib_2dcorners_lr[0].size(), stereo_calib_2dcorners_lr[1].size());
    CHECK_EQ(stereo_calib_2dcorners_lr[0].size(), stereo_calib_3d_coordinates.size());
    if (stereo_calib_2dcorners_lr[0].size() == 0) {
      LOG(ERROR) << "No images for stereo calib";
      return -1;
    }
    cv::Mat R, T, E, F;
    // stereoCalibrate must use Mat as input
    cv::Mat K0(duo_calib_param.Camera.cv_camK_lr[0]);
    cv::Mat dist0(duo_calib_param.Camera.cv_dist_coeff_lr[0]);
    cv::Mat K1(duo_calib_param.Camera.cv_camK_lr[1]);
    cv::Mat dist1(duo_calib_param.Camera.cv_dist_coeff_lr[1]);
    cv::stereoCalibrate(stereo_calib_3d_coordinates,
                        stereo_calib_2dcorners_lr[0],
                        stereo_calib_2dcorners_lr[1],
                        K0,
                        dist0,
                        K1,
                        dist1,
                        img_size,
                        R, T, E, F, CV_CALIB_FIX_INTRINSIC | CV_CALIB_RATIONAL_MODEL);
    cout << "stereo calib uses " << stereo_calib_3d_coordinates.size() << " img pairs "
         << endl << " R =\n" << R << endl << " T =\n" << T << endl;
    ss_log << "stereo calib uses " << stereo_calib_3d_coordinates.size() << " img pairs" << endl;

    // according to
    // http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereocalibrate
    // R and T is defined as C1_T_C0 [R T; 0001]
    duo_calib_param.Camera.D_T_C_lr[0] = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Cr_T_Cl = Eigen::Matrix4f::Identity();
    CHECK_EQ(R.type(), CV_64F);
    CHECK_EQ(T.type(), CV_64F);
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Cr_T_Cl(i, j) = R.at<double>(i, j);
      }
      Cr_T_Cl(i, 3) = T.at<double>(i, 0);
    }
    CHECK_EQ(duo_calib_param.Camera.D_T_C_lr.size(), 2);
    duo_calib_param.Camera.D_T_C_lr[1] = Cr_T_Cl.inverse();
  }

  // Concatenate the reprojection image before saving
  int img_size_width = img_size.width * camera_max_index;
  cv::Mat reproj_img(img_size.height, img_size_width, CV_8UC3);
  for (int lr = 0; lr < camera_max_index; ++lr) {
    res_debug_imgs[lr].copyTo(reproj_img(cv::Rect(lr * img_size.width, 0,
                                                  img_size.width, img_size.height)));
  }
  if (FLAGS_sensor_type == "UNKNOWN" ||
      FLAGS_sensor_type == "FACE") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::UNKNOWN;
  } else if (FLAGS_sensor_type == "XP") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XP;
  } else if (FLAGS_sensor_type == "XP2") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XP2;
  } else if (FLAGS_sensor_type == "XP3") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XP3;
  } else if (FLAGS_sensor_type == "XPIRL") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XPIRL;
  } else if (FLAGS_sensor_type == "XPIRL2") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XPIRL2;
  } else if (FLAGS_sensor_type == "XPIRL3") {
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::XPIRL3;
  } else {
    LOG(ERROR) << "Unsupport sensor type";
    duo_calib_param.sensor_type = XP::DuoCalibParam::SensorType::UNKNOWN;
  }
  duo_calib_param.device_id = FLAGS_device_id;
  std::string reproj_filename = "/tmp/calib_reproj.png";
  if (!FLAGS_save_calib.empty()) {
    boost::filesystem::path calib_path(FLAGS_save_calib);
    XP::save_camera_calib_param(FLAGS_save_calib, duo_calib_param);
    reproj_filename = calib_path.parent_path().string() + "/calib_reproj.png";
    std::ofstream out(calib_path.parent_path().string() + "/calib.log");
    out << ss_log.str();
    out.close();
  }
  if (!FLAGS_save_calib_yaml.empty()) {
    boost::filesystem::path calib_path(FLAGS_save_calib_yaml);
    reproj_filename = calib_path.parent_path().string() + "/calib_reproj.png";
    std::ofstream out(calib_path.parent_path().string() + "/calib.log");
    out << ss_log.str();
    out.close();
    if (!duo_calib_param.WriteToYaml(FLAGS_save_calib_yaml)) {
      LOG(ERROR) << "Save to " << FLAGS_save_calib_yaml << " failed";
    }
    if (FLAGS_sensor_type == "XPIRL3" || FLAGS_sensor_type == "XPIRL2") {
      XP::DuoCalibParam duo_calib_param_ir = duo_calib_param;
      for (int i = 0; i < 2; i++) {
        duo_calib_param_ir.Camera.cameraK_lr[i](0, 0) /= 2;
        duo_calib_param_ir.Camera.cameraK_lr[i](0, 2) /= 2;
        duo_calib_param_ir.Camera.cameraK_lr[i](1, 1) /= 2;
        duo_calib_param_ir.Camera.cameraK_lr[i](1, 2) /= 2;
      }
      duo_calib_param_ir.Camera.img_size /= 2;
      boost::filesystem::path calib_path(FLAGS_save_calib_yaml);
      std::string ir_calib_path = calib_path.parent_path().string()
                                  + "/ir_" + calib_path.filename().string();
      std::cout << "save ir calib yaml" << std::endl;
      if (!duo_calib_param_ir.WriteToYaml(ir_calib_path)) {
        LOG(ERROR) << "Save to " << FLAGS_save_calib_yaml << " failed";
      }
    }
  }
  cv::imwrite(reproj_filename, reproj_img);

  if (FLAGS_show_reproj) {
    if (FLAGS_display_timeout > 0) {
      // exit until any key press
      cv::waitKey(0);
    }
  }
  return 0;
}  // NOLINT
