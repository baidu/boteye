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

#ifndef NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_TYPE_H_
#define NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_TYPE_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <string>
#include <vector>
#include <chrono>

namespace Navigation {
struct WayPoint {
  float timestamp_sec;
  Eigen::Vector3f xyz;
  Eigen::Vector3f direction;
  char tag;

  WayPoint() :
    timestamp_sec(-1.f),
    direction(Eigen::Vector3f(0, 0, 0)),
    tag(0x00) {}

  bool operator <(const WayPoint& rhs) const {
    return timestamp_sec < rhs.timestamp_sec;
  }
};

typedef std::vector<WayPoint> VecWayPoint;

struct OAPathDirectionInfo {
  OAPathDirectionInfo() :
    direction(0, 0),
    tp(std::chrono::steady_clock::now()) {}
  OAPathDirectionInfo(const Eigen::Vector2f &init_direction,
                      const std::chrono::steady_clock::time_point init_tp) :
    direction(init_direction),
    tp(init_tp) {}

  Eigen::Vector2f direction;
  std::chrono::steady_clock::time_point tp;
};

struct ForceRelocPoseInfo {
  ForceRelocPoseInfo() : used(false) {}
  ForceRelocPoseInfo(const Eigen::Vector3f &input_pose,  // (x, y, direction)
                     const std::chrono::steady_clock::time_point &input_tp,
                     const bool input_used = false) :
    force_reloc_pose(input_pose),
    try_reloc_tp(input_tp),
    used(input_used) {}

  void updateTryRelocTimePoint(const std::chrono::steady_clock::time_point &tp
                               = std::chrono::steady_clock::now()) {
    try_reloc_tp = tp;
    used = true;
  }
  float secondsSinceLastReloc(const std::chrono::steady_clock::time_point &tp
                              = std::chrono::steady_clock::now()) {
    CHECK(used) << "Do not call this function when it has never been employed to force reloc.";
    return std::chrono::duration_cast<std::chrono::seconds>(tp - try_reloc_tp).count();
  }

  Eigen::Vector3f force_reloc_pose;
  std::chrono::steady_clock::time_point try_reloc_tp;
  bool used;
};

typedef std::vector<ForceRelocPoseInfo> VecForceRelocPoseInfo;

enum class NaviStatus {
  NORMAL = 0,
  LOST = 1,
  OBSTACLE_AVOID = 2,
  STOP = 3,
  STANDBY = 4,
  MANUAL = 5,
  LOST_RECOVERY = 6,
  FORCE_RELOC = 7,
  ROT_BEFORE_TRACK = 8,
};

enum class ForceRelocStatus {
  STOP = 0,
  ROTATE_TO_DIR = 1,
  WAIT_FOR_RELOC = 2,
  ROTATE_BACK = 3,
};

inline std::string NaviStatusToString(const NaviStatus navi_status) {
  if (navi_status == NaviStatus::NORMAL) {
    return "NORNAL";
  } else if (navi_status == NaviStatus::LOST) {
    return "LOST";
  } else if (navi_status == NaviStatus::LOST_RECOVERY) {
    return "LOST_RECOVERY";
  } else if (navi_status == NaviStatus::OBSTACLE_AVOID) {
    return "OBSTACLE_AVOID";
  } else if (navi_status == NaviStatus::STOP) {
    return "STOP";
  } else if (navi_status == NaviStatus::STANDBY) {
    return "STANDBY";
  } else if (navi_status == NaviStatus::MANUAL) {
    return "MANUAL";
  } else if (navi_status == NaviStatus::FORCE_RELOC) {
    return "FORCE_RELOC";
  } else if (navi_status == NaviStatus::ROT_BEFORE_TRACK) {
    return "ROT_BEFORE_TRACK";
  } else {
    return "UNKNOWN";
  }
}

/**
 * message for lidar
 */
// TODO(meng): maybe refactor radius_theta&x_y though typedef Vector2f
struct radius_theta{
  radius_theta() : radius(0), theta(0) {}
  radius_theta(const float init_radius, const float init_theta)
    : radius(init_radius), theta(init_theta) {}

  bool operator==(const radius_theta &input) {
    return radius == input.radius && theta == input.theta;
  }
  float radius;
  float theta;
};
struct x_y{
  x_y() : x(0), y(0) {}
  x_y(const float init_x, const float init_y) : x(init_x), y(init_y) {}

  bool operator==(const x_y &input) const {
    return x == input.x && y == input.y;
  }
  float x;
  float y;
};
struct XYHasher {
  size_t operator()(const x_y &input) const {
    return std::hash<float>()(input.x) ^ (std::hash<float>()(input.y) << 1);
  }
};

class ScanMessage {
  // [Note]: This class do not set scan_xy & scan_rt in the same time.
  // This class only does the conversion in specific getters.
  // If scan_rt is set by set_scan_rt(...), the range of theta is determined by the source data.
  // If scan_rt is changed or generated by other member functions, the range of theta is [-pi, pi].
  // The result of sort function is only in theta ascending order.
 public:
  ScanMessage() {}
  explicit ScanMessage(const std::chrono::steady_clock::time_point& input_tp)
    : sorted_by_theta_(false), tp_(input_tp) {}
  explicit ScanMessage(const std::vector<x_y>& input_scan_xy)
    : sorted_by_theta_(false), scan_xy_(input_scan_xy) {
  }
  explicit ScanMessage(const std::vector<radius_theta>& input_scan_rt)
    : sorted_by_theta_(false), scan_rt_(input_scan_rt) {
  }
  ScanMessage(const std::vector<x_y>& input_scan_xy,
              const std::chrono::steady_clock::time_point& input_tp)
    : sorted_by_theta_(false), scan_xy_(input_scan_xy), tp_(input_tp) {}
  ScanMessage(const std::vector<radius_theta>& input_scan_rt,
              const std::chrono::steady_clock::time_point& input_tp)
    : sorted_by_theta_(false), scan_rt_(input_scan_rt), tp_(input_tp) {}

  std::vector<x_y> get_scan_xy() const {
    return scan_xy_;
  }
  std::vector<radius_theta> get_scan_rt() const {
    return scan_rt_;
  }
  std::vector<x_y> get_scan_xy_if_empty_convert_from_rt() {
    if (scan_xy_.empty() && !scan_rt_.empty()) {
      convert_scan_rt_to_xy(scan_rt_, &scan_xy_);
    }
    return scan_xy_;
  }
  std::vector<radius_theta> get_scan_rt_if_empty_convert_from_xy(const bool need_sort = true) {
    if (scan_rt_.empty() && !scan_xy_.empty()) {
      convert_scan_xy_to_rt(scan_xy_, &scan_rt_);
      sorted_by_theta_ = false;
    }
    if (need_sort && !sorted_by_theta_) {
      theta_ascending_sort_scan_rt(&scan_rt_);
      sorted_by_theta_ = true;
    }
    return scan_rt_;
  }
  std::chrono::steady_clock::time_point get_time_point() const {
    return tp_;
  }
  // Generating float vector distributed by equidistant angles
  // The input angle must be in the range of [-M_PI, M_PI], counterclockwise
  std::vector<float> compute_standard_scan(const float begin_angle,
                                           const float end_angle,
                                           const size_t scan_size,
                                           const float invalid_val,
                                           const float delta_theta_ratio = 1.0) {
    std::vector<float> output_scan;
    output_scan.reserve(scan_size);
    float begin_angle_real = begin_angle;
    float end_angle_real = end_angle;
    std::vector<radius_theta> scan_rt = get_scan_rt_if_empty_convert_from_xy();
    if (scan_rt.empty()) {
      return output_scan;
    }

    // The angular range of scan_rt is in [-M_PI, M_PI]
    // Make the angle continuous
    for (auto& beam : scan_rt) {
      if (beam.theta < begin_angle)
        beam.theta += 2 * M_PI;
    }
    if (begin_angle >= end_angle)
      end_angle_real += 2 * M_PI;
    theta_ascending_sort_scan_rt(&scan_rt);
    const float angle_step = (end_angle_real - begin_angle_real) / (scan_size - 1);
    // Down sampling
    sample_smallest_r_within_delta_theta(angle_step * delta_theta_ratio, &scan_rt);
    int input_index = 0;
    for (int i = 0; i < scan_size; i++) {
      float lookup_angle = begin_angle_real + i * angle_step;
      // the angle to look up in lidar input is negative of this
      while (input_index < scan_rt.size() && scan_rt[input_index].theta < lookup_angle)
        input_index++;
      if (input_index >= scan_rt.size()) input_index = scan_rt.size() - 1;
      int input_index_prev = input_index - 1;
      if (input_index_prev < 0) input_index_prev = 0;
      // Make sure the offset is less than the preset value, otherwise assigning invalid val
      float angle_differences[2] = {
        static_cast<float>(fabsf(lookup_angle - scan_rt[input_index_prev].theta)),
        static_cast<float>(fabsf(lookup_angle - scan_rt[input_index].theta))};
      if (angle_differences[0] < angle_differences[1]
          && angle_differences[0] < 0.5 * angle_step * delta_theta_ratio) {
        output_scan.push_back(scan_rt[input_index_prev].radius);
      } else if (angle_differences[0] > angle_differences[1] &&
                 angle_differences[1] < 0.5 * angle_step * delta_theta_ratio) {
        output_scan.push_back(scan_rt[input_index].radius);
      } else {
        output_scan.push_back(invalid_val);
      }
    }
    return output_scan;
  }
  // Transform scan message from lidar coordinate to target coordinate
  Navigation::ScanMessage compute_scan_in_target_coordinate(const Eigen::Matrix4f T_XL) {
    Navigation::ScanMessage scan_msg_X;
    scan_msg_X.scan_reserve(scan_size());
    scan_msg_X.set_time_point(get_time_point());
    std::vector<x_y> scan_xy = get_scan_xy_if_empty_convert_from_rt();
    for (size_t i = 0; i < scan_xy.size(); i++) {
      float x = scan_xy[i].x;
      float y = scan_xy[i].y;
      Eigen::Vector4f valid_beam_L;
      valid_beam_L << x, y, 0, 1;
      Eigen::Vector4f point_O = T_XL * valid_beam_L;
      Navigation::x_y xy_A;
      xy_A.x = point_O[0];
      xy_A.y = point_O[1];
      scan_msg_X.scan_xy_push_back(xy_A);
    }
    return scan_msg_X;
  }

  void set_scan_xy(const std::vector<x_y> &input_scan_xy) {
    scan_xy_ = input_scan_xy;
    if (!scan_rt_.empty()) {
      scan_rt_.clear();
    }
  }
  void set_scan_rt(const std::vector<radius_theta> &input_scan_rt, const bool is_sorted = false) {
    scan_rt_ = input_scan_rt;
    if (!scan_xy_.empty()) {
      scan_xy_.clear();
    }
    sorted_by_theta_ = is_sorted;
  }
  void set_time_point(const std::chrono::steady_clock::time_point &input_tp) {
    tp_ = input_tp;
  }
  size_t scan_size() {
    return scan_xy_.empty() ? scan_rt_.size() : scan_xy_.size();
  }
  void scan_clear() {
    scan_xy_.clear();
    scan_rt_.clear();
    sorted_by_theta_ = false;
  }
  void scan_reserve(const size_t size) {
    scan_xy_.reserve(size);
    scan_rt_.reserve(size);
  }
  void scan_xy_push_back(const x_y &xy) {
    // this function will cause scan_rt_ empty
    scan_xy_.push_back(xy);
    if (!scan_rt_.empty()) {
      scan_rt_.clear();
    }
    sorted_by_theta_ = false;
  }
  void scan_rt_push_back(const radius_theta &rt, const bool in_order = false) {
    // this funtion will cause scan_xy_ empty
    scan_rt_.push_back(rt);
    if (!scan_xy_.empty()) {
      scan_xy_.clear();
    }
    sorted_by_theta_ = in_order;
  }

  static void sample_smallest_r_within_delta_theta(const float delta_theta,
                                                   std::vector<radius_theta> *scan_rt) {
    // [NOTE]: The input scan_rt should have been ascending sorted.
    const float half_delta_theta = 0.5 * delta_theta;
    std::vector<radius_theta> scan_rt_copy = *scan_rt;
    for (int i = 0; i < scan_rt_copy.size(); i++) {
      radius_theta& rt = (*scan_rt)[i];
      int j = i;
      while (j >= 0) {
        if (fabs(scan_rt_copy[j].theta - rt.theta) > half_delta_theta) {
          break;
        }
        rt.radius = fmin(rt.radius, scan_rt_copy[j].radius);
        j--;
      }
      j = i;
      while (j < scan_rt_copy.size()) {
        if (fabs(scan_rt_copy[j].theta - rt.theta) > half_delta_theta) {
          break;
        }
        rt.radius = fmin(rt.radius, scan_rt_copy[j].radius);
        j++;
      }
    }
  }

  static void theta_ascending_sort_scan_rt(std::vector<radius_theta> *scan_rt) {
    std::sort(scan_rt->begin(), scan_rt->end(),
              [](radius_theta first,  radius_theta second) { return first.theta < second.theta; });
  }

  static void convert_scan_rt_to_scan_v2f(const std::vector<radius_theta> &scan_rt,
                                          std::vector<Eigen::Vector2f> *scan_rt_v2f) {
    scan_rt_v2f->clear();
    scan_rt_v2f->reserve(scan_rt.size());
    for (auto &point_rt : scan_rt) {
      scan_rt_v2f->emplace_back(point_rt.radius, point_rt.theta);
    }
  }

  static void convert_scan_v2f_to_scan_rt(const std::vector<Eigen::Vector2f> &scan_rt_v2f,
                                          std::vector<radius_theta> *scan_rt) {
    scan_rt->clear();
    scan_rt->reserve(scan_rt_v2f.size());
    for (auto &point_rt_v2f : scan_rt_v2f) {
      scan_rt->emplace_back(point_rt_v2f(0), point_rt_v2f(1));
    }
  }

  static void convert_scan_xy_to_scan_v2f(const std::vector<x_y> &scan_xy,
                                          std::vector<Eigen::Vector2f> *scan_xy_v2f) {
    scan_xy_v2f->clear();
    scan_xy_v2f->reserve(scan_xy.size());
    for (auto &point_xy : scan_xy) {
      scan_xy_v2f->emplace_back(point_xy.x, point_xy.y);
    }
  }

  static void convert_scan_v2f_to_scan_xy(const std::vector<Eigen::Vector2f> &scan_xy_v2f,
                                          std::vector<x_y> *scan_xy) {
    scan_xy->clear();
    scan_xy->reserve(scan_xy_v2f.size());
    for (auto &point_xy_v2f : scan_xy_v2f) {
      scan_xy->emplace_back(point_xy_v2f(0), point_xy_v2f(1));
    }
  }

  static void convert_scan_xy_to_rt(const std::vector<x_y> &input_scan_xy,
                                    std::vector<radius_theta> *output_scan_rt) {
    output_scan_rt->clear();
    if (input_scan_xy.empty()) {
      return;
    }
    output_scan_rt->reserve(input_scan_xy.size());
    for (auto xy : input_scan_xy) {
      radius_theta rt;
      xy_to_rt(xy, &rt);
      output_scan_rt->push_back(rt);
    }
  }
  static void convert_scan_rt_to_xy(const std::vector<radius_theta> &input_scan_rt,
                                    std::vector<x_y> *output_scan_xy) {
    output_scan_xy->clear();
    if (input_scan_rt.empty()) {
      return;
    }
    output_scan_xy->reserve(input_scan_rt.size());
    for (auto rt : input_scan_rt) {
      x_y xy;
      rt_to_xy(rt, &xy);
      output_scan_xy->push_back(xy);
    }
  }
  static void xy_to_rt(const x_y &xy, radius_theta *rt) {
    rt->radius = std::hypotf(xy.x, xy.y);
    rt->theta = atan2f(xy.y, xy.x);  // [-pi, pi]
  }
  static void rt_to_xy(const radius_theta &rt, x_y *xy) {
    xy->x = rt.radius * cosf(rt.theta);
    xy->y = rt.radius * sinf(rt.theta);
  }

 protected:
  std::chrono::steady_clock::time_point tp_;
  std::vector<x_y> scan_xy_;
  std::vector<radius_theta> scan_rt_;
  bool sorted_by_theta_;  // Only in ascending order.
};

}  // namespace Navigation

#endif  // NAVIGATION_INCLUDE_NAVIGATION_NAVIGATION_TYPE_H_
