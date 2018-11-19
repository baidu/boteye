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
#ifndef XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_
#define XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_

#include <XP/helper/param.h>
#include <string>
#include <vector>
#include <limits>
namespace XP {

class AlgorithmParam : public ParamBase {
 public:
  AlgorithmParam() {
    Nature.gravity = Eigen::Vector3f(0, 0, -9.799);
  }

  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;
  struct FeatDetParam_t {
    int request_feat_num = 70;
    int pyra_level = 2;
    int fast_det_thresh = 10;
    int uniform_radius = 40;
    float min_feature_distance_over_baseline_ratio = 3;
    float max_feature_distance_over_baseline_ratio = 3000;
    int feature_track_length_thresh = 25;
    float feature_track_dropout_rate = 0.3f;
  } FeatDetParam;
  struct Mapping_Mode_t {
    bool use_vio_orientation_as_absolute = false;
    bool compute_cov = true;
    bool fix_latest_rig_at_ba = true;
    bool modify_prescan_map = false;
    bool ignore_feat_on_floor = true;
    bool fix_height_value = false;
    std::string feat_type = "nature";
    std::string mapper_type = "normal";
    std::string tag_family = "36h11";
    bool mirror_tag_detection = false;
  } MappingMode;
  struct Mapping_t {
    float max_feature_search_range = 20.f / 400.f;
    int min_keyrig_gap_num = 10;
    float min_keyrig_gap_time_sec = 0.5;
    int min_gen_local_map_gap_num = 10;
    float min_keyrig_dist = 0.05;
    float min_neighbour_bow_score = 0.1;
    float max_neighbour_rig_dis = 5.0;  // if a rig has high BOW score but larger spatial distance
                                        // than max_neighour_rig_dis, do not count it as key rig
    float orb_match_dist_thresh = 0.2 * 255;
    float orb_match_thresh_test_ratio = 0.8;
    int local_ba_min_movalble_rig_num = 1;
    int local_ba_max_movalble_rig_num = 5;
    int feat_det_num = 200;
    int feat_det_pyra_level = 2;
    int feat_det_uniform_radius = 20;
    int max_vio_fail_count_tolerance = 10;
    int short_term_vio_fail_count = 3;
    int max_reloc_attempt = 5;
    float loop_detect_cooldown_sec = 10;
    float loop_close_cooldown_sec = 10;
    float max_lost_sec_before_use_vio_to_reset = 3;
    float smoothing_time_length = 0.1;
    float calib_verify_stereo_good_match_ratio = 0.9;
    std::string slave_det = "direct";
  } Mapping;
  struct Tracking_t {
    float max_feature_search_range = 10.f / 400.f;
    float orb_match_dist_thresh = 0.3 * 255;
    float orb_match_thresh_test_ratio = 0.9;
    float feature_uncertainty = 5;
    bool use_of_id = true;
    bool use_april_tag = false;
    std::string slave_det = "direct";
    bool undistort_before_vio = true;
    bool negative_image = false;
  } Tracking;
  struct Reloc_t {
    float min_reloc_match_score = std::numeric_limits<float>::min();
    int min_3d_2d_num = 25;
    int min_ll_H_inlir_num = 10;
    float HScale_max = 2;
    float HScale_min = 0.5;
    float good_reproj_error = 0.025;  // 10 pixels with f = 400
    bool use_pnp = true;  // or relative pose
    float H_inlier_pixel_dis = 0.025;  // 10 pixels with f = 400
    float matches_area_ratio = 0.5;  // 0 - 1
    int small_error_count_thres = 25;  // equal or less than min_3d_2d_num
    bool check_with_noncentral_rel = true;  // or check_with_essential_matrix
    bool redo_bootstrap = true;  // re-bootstrap the query pose to the best ref pose
    bool use_input_rotation = false;  // only solve translation
    int H_max_plane_num = 3;
  } Reloc;
  struct Depth_t {
    int horizontal_bucket_num = 16;
    float height_below_cam = -0.5;
    float height_above_cam = 0.6;
    float depth_confidence = 0.2;
    float run_dense_stereo_freq = 10;
    float camera_to_floor_height = 0.1;
    std::string filter_type = "binning";
    bool use_stereo_sgbm = true;
  } Depth;
  struct Nature_t {
    Eigen::Vector3f gravity;
  } Nature;
  struct RosParam_t {
    bool enable_ros_topic = false;
    std::string slam_node = "xp_slam";
    std::string mapper_stable_topic = "mapper_stable";
    std::string slam_pose_topic = "slam_pose";
    std::string depth_image_topic = "depth_image";
    std::string slam_frame_id = "slam";
    Eigen::Matrix4f T_AD = Eigen::Matrix4f::Identity();
  } RosParam;
};

class NaviParam : public ParamBase {
 public:
  NaviParam() {}
  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;

  struct Navigation_t {
    std::string type = "navi";
    bool use_trajectory_file = false;
    std::string mode = "control";
    Eigen::Matrix4f T_AD = Eigen::Matrix4f::Identity();
    bool fuse_wheel_odom_pose = false;
    bool wheel_odom_vel_feedback = true;
    float base_velocity = 0.4;  // meter per second
    int grid_cell_size_x = 100;
    int grid_cell_size_y = 100;
    float grid_resolution = 0.05;
    int lost_recovery_timeout = 60;
    bool enable_obstacle_avoid = false;
    bool hierarchical_infalte = true;
    bool use_global_occ_grid_prior = false;
    float safe_obstacle_distance = 0.4;
    float valid_obstacle_dist_range = 2.0;
    float valid_obstacle_angle_range = M_PI / 3;
    float virtual_clear_range = 3.0;
    float road_width = 3;
    float control_frequency = 20;
    float update_frequency = 20;
  } Navigation;
  struct ActuatorConfig_t {
    std::string type = "reeman";
    std::string serial_dev = "/dev/ttyUSB0";
    bool reverse_heading = false;
    float encoder_ticks_num = 8000;
    float axis_diameter = 0.25;
    float wheel_diameter = 0.15;
    float device_radius = 0.3;
    uint8_t ultrasound_flag = 0;
    Eigen::Matrix4f T_AL = Eigen::Matrix4f::Identity();
    float lidar_obs_dist_thres = 0.5;
    float lidar_obs_angle_thres = M_PI_2;
    // only for ROS
    std::string odom_topic = "odom";
    std::string diag_topic = "diagnostics";
    std::string cmd_topic = "cmd_vel";
  } Actuator;
  struct LidarConfig_t {
    std::string type = "none";
    std::string serial_dev = "/dev/ttyUSB1";
    float min_valid_scan_dist = 0.2;
    // Only for ROS Scanner
    std::string scan_topic = "scan";
    std::string diag_topic = "diagnostics";
  } Lidar;
  struct Controller_t {
    std::string type = "PD";
    float track_distance = 0.4;
    float target_distance_tolerance = 0.1;
    float target_direction_tolerance = 3 * M_PI / 180;
    float inter_sample_resolution = 0.01;
    float v_max = 0.4;
    float abs_w_max = 0.8;
    float kp_v = 0.75;
    float kp_w = 0.51;
    float kp_acc_v = 0.5;
    float kp_acc_v_stop = 0.25;
    float kp_acc_w = 0.5;
    float velocity_multiplier = 1.0;
    float angular_multiplier = 1.0;
    float robot_radius_multiplier = 2.0;
  } Controller;
};

class ThreadParam : public ParamBase {
  static const int NO_BOUND_TO_CPU_CORE = -1;

 public:
  struct affinity_cpu_core {
    affinity_cpu_core() :
    mapper_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    frame_feat_det_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    pull_XP_imu_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    stream_XP_images_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    multiframe_consumer_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    draw_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    matching_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    optimization_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    heartBeatDetector_thread_cpuid(NO_BOUND_TO_CPU_CORE),
    visualization_cpuid(NO_BOUND_TO_CPU_CORE),
    vizReprojection_cpuid(NO_BOUND_TO_CPU_CORE),
    imuConsumer_cpuid(NO_BOUND_TO_CPU_CORE),
    publisher_cpuid(NO_BOUND_TO_CPU_CORE) {
    }
    void print_affinity_cpu_core() const {
      std::cout << "mapper cpu id:                     " << mapper_thread_cpuid << "\n";
      std::cout << "frame feature detection cpu id:    " << frame_feat_det_thread_cpuid << "\n";
      std::cout << "pull XP imu cpu id:                " << pull_XP_imu_thread_cpuid << "\n";
      std::cout << "steam XP images cpu id:            "
                << stream_XP_images_thread_cpuid << "\n";
      std::cout << "multi frame consumer loop cpu id:  "
                << multiframe_consumer_thread_cpuid << "\n";
      std::cout << "draw cpu id:                       " << draw_thread_cpuid << "\n";
      std::cout << "matching loop cpu id:              " << matching_thread_cpuid << "\n";
      std::cout << "optimization loop cpu id:          " << optimization_thread_cpuid << "\n";
      std::cout << "heart beat detector cpu id:        "
                << heartBeatDetector_thread_cpuid << "\n";
      std::cout << "publisher cpu id:                  " << publisher_cpuid << "\n";
      std::cout << "vizReprojection cpu id:            " << vizReprojection_cpuid << "\n";
      std::cout << "visualization cpu id:              " << visualization_cpuid << "\n";
      std::cout << "imu consumer cpu id:               " << imuConsumer_cpuid << "\n";
    }
    // Check user's settings, if it's invalid
    // corresponding cpu id will be reseted to default value: -1
    void check_affinity_cpu_core() {
      int max_cpu_cores = sysconf(_SC_NPROCESSORS_ONLN);
      if (static_cast<unsigned int>(mapper_thread_cpuid) >= max_cpu_cores) {
        mapper_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(frame_feat_det_thread_cpuid) >= max_cpu_cores) {
        frame_feat_det_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(pull_XP_imu_thread_cpuid) >= max_cpu_cores) {
        pull_XP_imu_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(stream_XP_images_thread_cpuid) >= max_cpu_cores) {
        stream_XP_images_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(multiframe_consumer_thread_cpuid) >= max_cpu_cores) {
        multiframe_consumer_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(draw_thread_cpuid) >= max_cpu_cores) {
        draw_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(matching_thread_cpuid) >= max_cpu_cores) {
        matching_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(optimization_thread_cpuid) >= max_cpu_cores) {
        optimization_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(heartBeatDetector_thread_cpuid) >= max_cpu_cores) {
        heartBeatDetector_thread_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(publisher_cpuid) >= max_cpu_cores) {
        publisher_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(vizReprojection_cpuid) >= max_cpu_cores) {
        vizReprojection_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(visualization_cpuid) >= max_cpu_cores) {
        visualization_cpuid = NO_BOUND_TO_CPU_CORE;
      }
      if (static_cast<unsigned int>(imuConsumer_cpuid) >= max_cpu_cores) {
        imuConsumer_cpuid = NO_BOUND_TO_CPU_CORE;
      }
    }
    // frontend
    int mapper_thread_cpuid;
    int frame_feat_det_thread_cpuid;
    int pull_XP_imu_thread_cpuid;
    int stream_XP_images_thread_cpuid;
    int multiframe_consumer_thread_cpuid;
    int draw_thread_cpuid;
    // backend
    int matching_thread_cpuid;
    int optimization_thread_cpuid;
    // other
    int heartBeatDetector_thread_cpuid;
    int publisher_cpuid;
    int vizReprojection_cpuid;
    int visualization_cpuid;
    int imuConsumer_cpuid;
  };

 public:
  ThreadParam() {
  }
  bool LoadFromYaml(const std::string& filename) override;
  bool WriteToYaml(const std::string& filename) override;
  bool LoadFromCvYaml(const std::string& filename) override;
  bool WriteToCvYaml(const std::string& filename) override;
  void PrintThreadParam(void) const {
    m_affinity_cpu_core.print_affinity_cpu_core();
  }
  void CheckThreadParam() {
    m_affinity_cpu_core.check_affinity_cpu_core();
  }
  inline int get_mapper_cpu_id() const {
    return m_affinity_cpu_core.mapper_thread_cpuid;
  }
  inline int get_frame_feat_det_cpu_id() const {
    return m_affinity_cpu_core.frame_feat_det_thread_cpuid;
  }
  inline int get_pull_li_img_cpu_id() const {
    return m_affinity_cpu_core.pull_XP_imu_thread_cpuid;
  }
  inline int get_stream_li_images_cpu_id() const {
    return m_affinity_cpu_core.stream_XP_images_thread_cpuid;
  }
  inline int get_multi_frame_consumer_loop_cpu_id() const {
    return m_affinity_cpu_core.multiframe_consumer_thread_cpuid;
  }
  inline int get_draw_cpu_id() const {
    return m_affinity_cpu_core.draw_thread_cpuid;
  }
  inline int get_matching_loop_cpu_id() const {
    return m_affinity_cpu_core.matching_thread_cpuid;
  }
  inline int get_optimization_loop_cpu_id() const {
    return m_affinity_cpu_core.optimization_thread_cpuid;
  }
  inline int get_hearbeat_detector_cpuid() const {
    return m_affinity_cpu_core.heartBeatDetector_thread_cpuid;
  }
  inline int get_visualization_cpuid() const {
    return m_affinity_cpu_core.visualization_cpuid;
  }
  inline int get_imu_consumer_cpuid() const {
    return m_affinity_cpu_core.imuConsumer_cpuid;
  }
  inline int get_publisher_cpuid() const {
    return m_affinity_cpu_core.publisher_cpuid;
  }
  inline int get_viz_reprojection_cpuid() const {
    return m_affinity_cpu_core.vizReprojection_cpuid;
  }

 private:
  affinity_cpu_core m_affinity_cpu_core;
};
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_PARAM_INTERNAL_H_
