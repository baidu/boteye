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
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <XP/util/calibration_utils.h>
#include <XP/depth/depth_utils.h>
#include <Eigen/LU>  // For Matrix4f inverse
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <limits>
#include <memory>
#include <numeric>  // for std::accumulate

using std::cout;
using std::endl;
using std::vector;

typedef vector<cv::Point2f> DetectedCorners;
typedef vector<cv::Point3f> ObjectCorners;

// Get a copy of input vector v as we will do sorting on it.
void compute_mean_median_stddev(std::vector<float> v,
                                const std::string& depth_method,
                                float* mean, float* median, float* stddev) {
  CHECK(!v.empty());
  float sum = std::accumulate(v.begin(), v.end(), 0.f);
  *mean = sum / v.size();
  vector<float> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 [mean](float x) { return x - *mean; });
  float sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.f);
  *stddev = std::sqrt(sq_sum / v.size());
  std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
  *median = v[v.size()/2];
}

cv::Mat compute_stereo_depth(const XP::DuoCalibParam& calib_param,
                             const std::string& depth_method,
                             const vector<vector<boost::filesystem::path> >& file_names_vec_lr,
                             const int img_pair_id) {
  CHECK_EQ(file_names_vec_lr[0].size(), file_names_vec_lr[1].size());
  CHECK_LT(img_pair_id, file_names_vec_lr[0].size());

  cv::Mat img_l = cv::imread(file_names_vec_lr[0][img_pair_id].string(), CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat img_r = cv::imread(file_names_vec_lr[1][img_pair_id].string(), CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat disparity_img;
  vector<cv::Mat> disparity_ml;
  cv::Mat disparity_buf;  // for filterSpeckles

  if (depth_method == "multilevel_bm") {
    XP::multilevel_stereoBM(calib_param,
                            img_l,
                            img_r,
                            &disparity_img,
                            &disparity_ml,
                            0,
                            2,
                            &disparity_buf);
  } else if (depth_method == "twolevel_bm") {
    XP::twolevel_stereoBM(calib_param,
                          img_l,
                          img_r,
                          &disparity_img,
                          &disparity_ml,
                          &disparity_buf);
  } else {
    LOG(FATAL) << "Non-supported depth method: " << depth_method;
  }
  CHECK_EQ(disparity_img.type(), CV_16SC1);
  CHECK_GT(disparity_img.rows, 0);
  CHECK_GT(disparity_img.cols, 0);
  return disparity_img;
}

inline int bin_id(float val, float min_val, float bin_unit, int bin_num) {
  const int id = static_cast<int>((val - min_val) / bin_unit);
  return std::min(std::max(id, 0), bin_num - 1);
}
bool check_calibration_with_depth(const vector<vector<DetectedCorners>>& stereo_calib_2dcorners_lr,
                                  const vector<ObjectCorners>& stereo_calib_3d_coordinates,
                                  const vector<int> stereo_img_ids,
                                  const XP::DuoCalibParam& calib_param,
                                  const std::string& depth_method,
                                  const vector<vector<boost::filesystem::path> >& file_names_vec_lr,
                                  std::stringstream* ss_log) {
  vector<float> depth_ratio_triangulate, depth_ratio_disparity;
  depth_ratio_triangulate.reserve(3000);
  depth_ratio_disparity.reserve(3000);

  // collect histogram statistics
  constexpr float min_dist = 0.2;
  constexpr float bin_unit = 0.2;
  constexpr int bin_num = 20;
  struct NumAndRatio {
    int count = 0;
    float triangulation_result_ratio_sum = 0;
    float disp_lut_result_ratio_sum = 0;
  };
  // We only store 20 bins in the distance and stats histogram
  std::vector<NumAndRatio> dist_histogram(bin_num);

  std::ofstream of;
  of.open("/tmp/point_clouds.csv", std::ios::out);
  // Further check the calibration with
  // Method 1) april tag R,t -> direct corner 3D positions
  // Method 2) detected corners -> undistort ray -> triangulate 3D positions
  // Method 3) detected corners -> rectified 2D points -> disparity lookup -> 3D positions
  for (int stereo_it = 0; stereo_it < stereo_calib_3d_coordinates.size(); stereo_it++) {
    const DetectedCorners& corners_l = stereo_calib_2dcorners_lr[0][stereo_it];
    const DetectedCorners& corners_r = stereo_calib_2dcorners_lr[1][stereo_it];
    const ObjectCorners& pts_3d_B = stereo_calib_3d_coordinates[stereo_it];  // in {B}oard coord
    CHECK_EQ(corners_l.size(), corners_r.size());
    CHECK_EQ(corners_l.size(), pts_3d_B.size());

    // Method 1) from board R,t
    // TODO(mingyu): Only estimate the board Rt from the left view w/o RANSAC for now.
    cv::Mat C_om_B, C_t_B, C_R_B;
    cv::solvePnP(pts_3d_B,
                 corners_l,
                 calib_param.Camera.cv_camK_lr[0],
                 calib_param.Camera.cv_dist_coeff_lr[0],
                 C_om_B, C_t_B);  // C_om_B, C_t_B all in type CV_64F (double)
    cv::Rodrigues(C_om_B, C_R_B);
    // verify reproj error
    vector<cv::Point2f> corner_reproj;
    cv::projectPoints(pts_3d_B,
                      C_om_B,
                      C_t_B,
                      calib_param.Camera.cv_camK_lr[0],
                      calib_param.Camera.cv_dist_coeff_lr[0],
                      corner_reproj);
    CHECK_EQ(corner_reproj.size(), corners_l.size());
    float reproj_error_sum = 0;
    int reproj_error_count = 0;
    for (size_t it_corner = 0; it_corner < corner_reproj.size(); it_corner++) {
      float x_diff = corners_l[it_corner].x - corner_reproj[it_corner].x;
      float y_diff = corners_l[it_corner].y - corner_reproj[it_corner].y;
      float diff = x_diff * x_diff + y_diff * y_diff;
      reproj_error_sum += sqrt(diff);
      ++reproj_error_count;
    }
    // in stereo calibration, we set 2 as threshold for filtering largest reproj-error
    if (reproj_error_sum / reproj_error_count > 2) {
      cout << "check_calibration_with_depth img " << stereo_it
           << " solvePnP avg repoj error too big " << reproj_error_sum / reproj_error_count << endl;
      *ss_log << "check_calibration_with_depth img " << stereo_it
              << " solvePnP avg repoj error too big "
              << reproj_error_sum / reproj_error_count << endl;
    }
    // Method 2)
    // Undistort detected corners to the unit plane and then do triangulation
    DetectedCorners undist_corners_l, undist_corners_r;  // Undistort to the unit plane
    cv::undistortPoints(corners_l,
                        undist_corners_l,
                        cv::Mat(calib_param.Camera.cv_camK_lr[0]),
                        calib_param.Camera.cv_dist_coeff_lr[0]);
    cv::undistortPoints(corners_r,
                        undist_corners_r,
                        cv::Mat(calib_param.Camera.cv_camK_lr[1]),
                        calib_param.Camera.cv_dist_coeff_lr[1]);
    vector<cv::Matx34f> proj_mat_lr(2);
    for (int lr = 0; lr < 2; lr++) {
      Eigen::Matrix4f C_T_W = calib_param.Camera.D_T_C_lr[lr].inverse();
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
          proj_mat_lr[lr](i, j) = C_T_W(i, j);
        }
      }
    }
    cv::Mat_<float> homo_pnts_3d;
    cv::triangulatePoints(proj_mat_lr[0], proj_mat_lr[1],
                          undist_corners_l, undist_corners_r, homo_pnts_3d);
    CHECK_EQ(corners_l.size(), homo_pnts_3d.cols);

    // Method 3)
    // Rectify detected corners on the left view to lookup on the disparity map
    DetectedCorners rectify_corners_l;
    cv::undistortPoints(corners_l,
                        rectify_corners_l,
                        cv::Mat(calib_param.Camera.cv_camK_lr[0]),
                        calib_param.Camera.cv_dist_coeff_lr[0],
                        calib_param.Camera.stereo_rectify_R_lr[0],
                        calib_param.Camera.cv_undist_K_lr[0]);
    CHECK_EQ(rectify_corners_l.size(), corners_l.size());
    cv::Mat disparity = compute_stereo_depth(calib_param,
                                             depth_method,
                                             file_names_vec_lr,
                                             stereo_img_ids[stereo_it]);

    // Evaluate 3D position computed by different methods
    vector<ObjectCorners> pts_set(3);
    for (int i = 0; i < 3; i++) {
      pts_set[i].reserve(corners_l.size());
    }
    for (int pt_it = 0; pt_it < corners_l.size(); pt_it++) {
      // Method 1) from board R,t
      cv::Mat pt_B(3, 1, CV_64F);
      pt_B.at<double>(0) = pts_3d_B[pt_it].x;
      pt_B.at<double>(1) = pts_3d_B[pt_it].y;
      pt_B.at<double>(2) = pts_3d_B[pt_it].z;
      cv::Mat pt_C_m1_mat = C_R_B * pt_B + C_t_B;
      cv::Point3f pt_C_m1(pt_C_m1_mat.at<double>(0),
                          pt_C_m1_mat.at<double>(1),
                          pt_C_m1_mat.at<double>(2));

      // Method 2) from triangulation
      cv::Point3f pt_C_m2(homo_pnts_3d(0, pt_it) / homo_pnts_3d(3, pt_it),
                          homo_pnts_3d(1, pt_it) / homo_pnts_3d(3, pt_it),
                          homo_pnts_3d(2, pt_it) / homo_pnts_3d(3, pt_it));

      // Method 3) from disparity lookup
      // [NOTE] It is possible that the corner does *NOT* exist in the disparity map
      int x = static_cast<int>(rectify_corners_l[pt_it].x + 0.5);
      int y = static_cast<int>(rectify_corners_l[pt_it].y + 0.5);
      int16_t disp = 0;
      if (x >= 0 && x < calib_param.Camera.img_size.width &&
          y >= 0 && y < calib_param.Camera.img_size.height) {
        disp = disparity.at<int16_t>(y, x);
      }
      if (disp <= 0) {
        continue;
      }
      cv::Vec4f xyz_homo =
          calib_param.Camera.Q * cv::Vec4f(x, y, static_cast<float>(disp) / 16.f, 1);
      cv::Vec3d pt_C_rect(xyz_homo[0] / xyz_homo[3],
                          xyz_homo[1] / xyz_homo[3],
                          xyz_homo[2] / xyz_homo[3]);
      // Convert back to the original C_left coordinate before rectification
      cv::Mat pt_C_m3_mat =
          calib_param.Camera.stereo_rectify_R_lr[0].t() * cv::Mat(pt_C_rect);
      cv::Point3f pt_C_m3(pt_C_m3_mat.at<double>(0),
                          pt_C_m3_mat.at<double>(1),
                          pt_C_m3_mat.at<double>(2));

      const float triang_over_pnp = pt_C_m2.z / pt_C_m1.z;
      const float disp_over_pnp = pt_C_m3.z / pt_C_m1.z;
      // Logging
      VLOG(2) << "stereo_it: " << stereo_it << " pt_it: " << pt_it << "\n"
              << "pt_C_m1: " << pt_C_m1.x << " " << pt_C_m1.y << " " << pt_C_m1.z << "\n"
              << "pt_C_m2: " << pt_C_m2.x << " " << pt_C_m2.y << " " << pt_C_m2.z << "\n"
              << "pt_C_m3: " << pt_C_m3.x << " " << pt_C_m3.y << " " << pt_C_m3.z;

      // The format for csv:
      // {m1_x, m1_y, m1_z, m2_x, m2_y, m2_z, m3_x, m3_y, m3_z, m2_z/m1_z, m3_z/m1_z
      of << pt_C_m1.x << ", " << pt_C_m1.y << ", " << pt_C_m1.z << ", "
         << pt_C_m2.x << ", " << pt_C_m2.y << ", " << pt_C_m2.z << ", "
         << pt_C_m3.x << ", " << pt_C_m3.y << ", " << pt_C_m3.z << ", "
         << triang_over_pnp << ", " << disp_over_pnp << endl;

      pts_set[0].push_back(pt_C_m1);
      pts_set[1].push_back(pt_C_m2);
      pts_set[2].push_back(pt_C_m3);

      depth_ratio_triangulate.push_back(triang_over_pnp);
      depth_ratio_disparity.push_back(disp_over_pnp);

      // accumulate histogram
      int id = bin_id(pt_C_m1.z, min_dist, bin_unit, bin_num);
      dist_histogram[id].count++;
      dist_histogram[id].triangulation_result_ratio_sum += triang_over_pnp;
      dist_histogram[id].disp_lut_result_ratio_sum += disp_over_pnp;
    }
  }
  of.close();

  if (depth_ratio_triangulate.empty() || depth_ratio_disparity.empty()) {
    // Should not really happen!
    return false;
  }
  // Compute the stats of z ratio, i.e., m2_z/m1_z and m3_z/m1_z
  float mean, median, stddev;
  compute_mean_median_stddev(depth_ratio_triangulate, depth_method, &mean, &median, &stddev);
  cout << "depth ratio (triangulation / board R,t from pnp): median / mean (stddev) = "
            << median << " / " << mean << " (" << stddev << ")\n";
  *ss_log << "depth ratio (triangulation / board R,t from pnp): median / mean (stddev) = "
          << median << " / " << mean << " (" << stddev << ")\n";
  compute_mean_median_stddev(depth_ratio_disparity, depth_method, &mean, &median, &stddev);
  cout << "depth ratio (disparity lut / board R,t from pnp): median / mean (stddev) = "
            << median << " / " << mean << " (" << stddev << ")\n";
  *ss_log << "depth ratio (disparity lut / board R,t from pnp): median / mean (stddev) = "
          << median << " / " << mean << " (" << stddev << ")\n";
  // display histogram
  cout << "distance ratio histogram:\n"
       << "[dist]: pnt#, avg triangulation ratio, avg disp lut ratio"
       << endl;
  *ss_log << "distance ratio histogram:\n"
          << "[dist]: pnt#, avg triangulation ratio, avg disp lut ratio"
          << endl;
  for (size_t i = 0; i < bin_num; ++i) {
    if (i == 0) {
      cout << "[0 - " << min_dist + bin_unit << "]: ";
      *ss_log << "[0 - " << min_dist + bin_unit << "]: ";
    } else if (i < bin_num - 1) {
      cout << "[" << min_dist + i * bin_unit << " - " << min_dist + (i + 1) * bin_unit << "]: ";
      *ss_log << "[" << min_dist + i * bin_unit << " - " << min_dist + (i + 1) * bin_unit << "]: ";
    } else {
      cout << "[" << min_dist + i * bin_unit << " - inf]:";
      *ss_log << "[" << min_dist + i * bin_unit << " - inf]:";
    }
    if (dist_histogram[i].count == 0) {
      cout    << "0, -, -" << endl;
      *ss_log << "0, -, -" << endl;
    } else {
      cout << dist_histogram[i].count << ", "
           << dist_histogram[i].triangulation_result_ratio_sum / dist_histogram[i].count << ", "
           << dist_histogram[i].disp_lut_result_ratio_sum / dist_histogram[i].count << endl;
      *ss_log << dist_histogram[i].count << ", "
              << dist_histogram[i].triangulation_result_ratio_sum / dist_histogram[i].count << ", "
              << dist_histogram[i].disp_lut_result_ratio_sum / dist_histogram[i].count << endl;
    }
  }
  return true;
}
