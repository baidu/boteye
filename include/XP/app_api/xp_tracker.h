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

#ifndef __XP_TRACKER_H__  // NOLINT
#define __XP_TRACKER_H__  // NOLINT
#include <XP/app_api/pose_packet.h>
#include <XP/data_atom/basic_datatype.h>
#include <XP/worker/actuator.h>
#include <XP/helper/param.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace XP_TRACKER {

/**
 * \brief This is an experimental function that requires Intel Neural Compute Stick (NCS) to be
 *        inserted to the host device and NCSDK installed.
 * \param output_layer_string The name of the output layer name of the input network (graph).
 *                            Currently only support "Softmax" and "DetectionOutput".
 * \param graph_filename The full file path to the graph file (binary) to be loaded to NCS.
 * \param network_image_width The network input image width
 * \param network_image_height The network input image height
 * \param network_means  The means per channel (total 3) to preprocess the input image
 * \param network_scales The scale per channel (total 3) to preprocess the input image
 * \return success or not.  Always return false if NCSDK is not installed or NCS is not found.
 */
bool init_ncs_worker(const std::string& output_layer_string,
                     const std::string& graph_filename,
                     const int network_image_width,
                     const int network_image_height,
                     const float network_means[],
                     const float network_scales[]);
/**
 * \brief Set the canvas for drawing the NCS result.
 * \param canvas Pre-allocated cv::Mat CV_8UC3 for visualization of the NCS result.
 */
bool set_ncs_canvas(cv::Mat* canvas);

/**
 * \brief Initialize the sensor to run with a live sensor.  The user is responsible to properly
 *        initilize the live sensor, register the data callbacks, and start/stop running.
 * \return success or not
 */
bool init_live_sensor();

/**
 * \brief Initialzie the data loader that loads images and imu from a folder.
 *        The data loader will disable all kinds of live sensor
 * \param folder_path where to load the data (similar to record_path)
 * \return success or not
 */
bool init_data_loader(const std::string& folder_path);

/**
 * \brief Initialize tracking algorithms
 * \param vio_config Config files controling vio algorithm
 * \param bow_dic_path Bag of words file to load
 * \param cam_calib_file_path Camera calibration file to load.  If empty, camera calibration
 *        will be loaded from DUO device directly.
 * \param use_fast_feat Use fast (or harris) for feature detection
 * \param use_iba_for_vio Use IBA as the backend for VIO
 * \return success or not
 */
bool init_tracker(const std::string& vio_config,
                  const std::string& bow_dic_path,
                  const std::string& depth_param_path,
                  const std::string& cam_calib_file_path,
                  const bool use_fast_feat,
                  const bool use_iba_for_vio);
/**
 * \brief Initialize udp service for poses. Will listen from any IP address at
 *        the specified port, and stream poses after handshake.
 * \param port port of host machine to send udp package
 * \return success or not
 */
bool init_udp_service(int port);
/**
 * \brief Initialize udp end point and stream pose to the specified ip & port.
 * \param udp_ip ip address to send udp package
 * \param port port of host machine to send udp package
 * \return success or not
 */
bool init_udp_stream(const std::string& udp_ip, int port);
/**
 * \brief Initialize udp listen to UDP port
 * \param port port to listen to
 * \return success or not
 */
bool init_udp_listen(int port);
/**
 * \brief Initialize robot client to log and get commands from server
 * \param server_address address of server
 * \return success or not
 */
bool init_robot_client(const std::string &server_address,
                       const std::string &stream_address,
                       const std::string &device_id,
                       unsigned int stream_width,
                       unsigned int stream_height);
/**
 * \brief Load map data (in pb format) before run tracker.
 * \param pb_path protobuf file path
 * \return success or not
 */
bool load_map(const std::string& pb_path);
/**
 * \brief Save map data (in pb format) after tracker finishes
 * \param pb_path protobuf file path
 * \return success or not
 */
bool save_map(const std::string& pb_path);
/**
 * \brief Set the record path for the current live tracking session before calling run_tracker_MT.
 * \param record_path record file path (the base folder)
 * \param calib_file sensor calibration yaml file path, which will also be saved to record_path
 * \param record_map_only If true, we only record the map w/o the raw data (images + imu)
 * \return The real record path
 */
std::string set_record_path(const std::string& record_path,
                            const std::string& calib_path,
                            const bool record_map_only);
/**
 * \brief Set canvas for drawing top down view.
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 *        clear_canvas_before_plot Whether or not clear the input canvas before plotting
 * \return success or not
 */
bool set_top_view_canvas(cv::Mat* canvas, bool clear_canvas_before_plot);
/**
 * \brief Set canvas for drawing camera view.
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 * \param view_num number of views to show. Can be 1 (only left view) or 2 (both left & right views)
 * \return success or not
 */
bool set_camera_view_canvas(cv::Mat* canvas, int view_num = 1);
/**
 * \brief Save draw canvas for depth image.
 *        If this is unset, the depth computation will be disabled (which saves ~100% cpu).
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 * \return success or not
 */
bool set_depth_view_canvas(cv::Mat* canvas);

/**
 * TODO(mingyu): Put back descriptions
 */
bool set_navigator(std::shared_ptr<XP::Actuator> actuator,
                   const std::string& navigation_folder,
                   const XP::NaviParam &navi_param);

/**
 * TODO(mingyu): Put back descriptions
 */
bool set_navigator_trajectory(const std::string &navigation_folder);

/**
 * TODO(mingyu): Put back descriptions
 */
bool set_navigator_mouse_data(const XP::MouseData& mouse_data);

/**
 * set navigator dest waypoint.
 * @param x x of slam coordinate
 * @param y y of slam coordinate
 * @return true if set dest xy succeed
 */
bool set_navigator_dest_xy(float world_x, float world_y);

/**
 * set navigator motion mode.
 * @param navigation motion mode
 * @return true if set motion mode succeed
 */
bool set_navigator_motion_mode(const XP_TRACKER::MotionMode& motion_mode);

/**
 * set navigator manual motion action.
 * @param navigation manual motion action
 * @return true if set manual motion action succeed
 */
bool set_navigator_manual_action(const XP_TRACKER::ManualMotionAction& motion_action);

/**
 * when navigator in loop mode, call this function to finish set loop target waypoints.
 * @return true if set succedd
 */
bool finish_set_loop_targets();

/**
 * \brief Save draw canvas for navigation control image.
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 * \return success or not
 */
bool set_navi_canvas(cv::Mat* canvas);

/**
 * @brief send_command_to_mapper send a command to mapper. Check mapper implementation for
 *        supported commands, e.g. "AddTagToLastKeyFrame"
 * @param command e.g. "AddTagToLastKeyFrame"
 * @param val e.g. a letter if the command is AddTagToLastKeyFrame
 * @param ret_ptr a pointer for return val. Pass nullptr if not important
 * @return success or not
 */
bool send_command_to_mapper(const std::string& command, uint32_t val, uint32_t* ret_ptr = nullptr);
/**
 * \brief Run tracking in multi-threaded fashion (requires 200% CPU in PC)
 * \return success or not
 */
bool run_tracker_MT();
/**
 * \brief Stop the multi-threaded program properly
 * \return success or not
 */
bool stop_tracker();
/**
 * \brief Draw all the pre-set canvas once
 */
bool draw_once();
/**
 * @brief check if duo_vio_tracker is still running
 * @return true for running status
 */
bool is_duo_vio_tracker_running(void);

typedef std::function<void(const cv::Mat&, const cv::Mat&, float)> StereoImagesCallback;
/**
 * \brief Register callback functions for getting raw imgs
 * \param callback the call back function if a new image pair is ready. This call back will
 *                 BLOCK the whole sensor image pulling thread, so callback has to be very
 *                 light. This function only works with LI sensor mode
 */
bool set_stereo_images_callback(const StereoImagesCallback& callback);

/**
 * \brief Callback function for getting pose, speed, angular speed and feature nums for MAV
 * \note  do NOT do a large function here as it can potentially block the internal vioCallback function,
 *        which can impact the tracking performance.
 */
typedef std::function<void(const VioState& vio_state)> VioStateCallback;
/**
 * \brief Register callback functions for getting vio state
 */
bool set_vio_state_callback(const VioStateCallback& callback);

/**
 * \brief Get 2d pose based on mapper result
 *        If no previous map is loaded, [0, 0] is defined as the initial position of the sensor.
 *        If a previous map is loaded, [0, 0] and orientation is inherited from the previous map.
 * \param x A pointer for saving x. X axis is defined as right hand direction of the sensor.
 * \param y A pointer for saving y. Y axis is defined as the frontal direction of the sensor.
 * \param yaw A pointer for saving yaw value. The range of yaw is (-pi, pi].
 *            yaw is the clock-wise angle from the camera front axis.
 *            yaw = atan2(cam_z_in_W(0), cam_z_in_W(1))
 *            cam_z_in_W = W_R_Cl * Vector3f(0, 0, 1)
 * \return success or not
 */
bool get_tracker_latest_2d_pose(float* x, float* y, float* yaw);
/**
 * \brief Get 3d pose based on mapper result
 *        If no previous map is loaded, [0, 0, 0] is defined as the initial position.
 *        Z is up, X is right, Y is front.
 *        If a previous map is loaded, positon and orientation is inherited from the previous map.
 * \param W_T_D_4x4 4x4 row matrix. It is the transformation between World and Device,
 *                  following A_T_B definition. W_T_D_4x4 has to be preallocated
 * \return success or not
 */
bool get_mapper_latest_3d_pose(float* W_T_D_4x4);
/**
 * \brief Get covriance matrix of the position result (xyz)
 * 				If no previous map is loaded, always return false
 *        The covariance matrix will grow constantly if reloc is not successful
 *        Ideally reloc can be done for every new input image pairs.
 *        In reality reloc can be done for every few input image pairs.
 * \param cov_3x3 3x3 row matrix. It is the covariance of the xyz components of pose
 * \return success or not
 */
bool get_mapper_reloc_xyz_cov(float* cov_3x3);

/**
 * \brief Get 3d pose based on vio result
 *        If no previous map is loaded, [0, 0, 0] is defined as the initial position.
 *        Z is up, X is right, Y is front.
 *        If a previous map is loaded, positon and orientation is inherited from the previous map.
 * \param W_T_D_4x4 4x4 row matrix. It is the transformation between World and Device,
 *        following A_T_B definition. W_T_D_4x4 has to be preallocated.
 * \return success or not
 */
bool get_vio_latest_3d_pose(float* W_T_D_4x4);

/**
 * \brief Get camera intrinsics
 * \param lr left (0) or right(1) camera
 * \param K 3x3 row matrix. K has to be preallocated.
 * \return success or not
 */
bool get_cam_K(int lr, float* K);
/**
 * \brief Get the position of key rigs in world coordinate system
 *        Note the position of old rigs may change if loop closure happens
 * \param rigs_xyz_ptr pointer of CV_32FC3. 1 x N Mat where N is the
 *        number of key rigs (from new to old)
 * \return success or not
 */
bool get_mapper_key_rigs_pos(cv::Mat* rigs_xyz_ptr);
/**
 * \brief Get the position and yaw angle of key rigs in world coordinate system
 *        Note the pose of old rigs may change if loop closure happens
 * \param rigs_xy_yaw_ptr pointer of CV_32FC3. 1 x N Mat where N is the
 *        number of key rigs (from new to old).
 *        Each element of rigs_xy_yaw is cv::Vec3f(x, y, yaw)
 *        The definition of x y yaw is the same as in get_tracker_latest_2d_pose
 * \return success or not
 */
bool get_mapper_key_rigs_xy_yaw(cv::Mat* rigs_xy_yaw_ptr);
/**
 * \brief Get the *prescan* xy position and yaw angle of key rigs in world cooridnate system
 *        Note that the poses of rigs are loaded from a pre-built map (with -pb_load option),
 *        and will not change.
 * \param rigs_xy_yaw_ptr pointer of CV_32FC3.  1 x N Mat where N is the
 *        number of key rigs (from new to old).
 *        Each element of rigs_xy_yaw is cv::Vec3f(x, y, yaw)
 *        The definition of x y yaw is the same as in get_tracker_latest_2d_pose
 */
bool get_prescan_key_rigs_xy_yaw(cv::Mat* rigs_xy_yaw_ptr);
/**
 * \brief Get the depth img computed from stereo camera
 *        (0, 0, 0) is set for pixels that can not find a proper depth value
 * \param depth_img_ptr pointer of depth image
 * \return success or not
 */
bool get_depth_img(cv::Mat_<cv::Vec3f>* depth_img_ptr);
/**
 * \brief The status of tracking engine
 */
struct TrackingEngineStatus {
  bool vio_lost;                // Wether or not the vio engine gets lost
  float current_time;           // The current second
  float last_mapper_pose_time;  // The second of the last available pose of mapper
  float last_vio_pose_time;     // The second of the last available pose of vio
};
/**
 * \brief Get the status of tracking engine
 * \param status_ptr pointer of TrackingEngineStatus
 * \return success or not
 */
bool get_tracking_engine_status(TrackingEngineStatus* status_ptr);
/**
 * \brief Inform tracking engine that the tracking is lost
 *        Although tracking engine applies many rules to determine if it gets lost,
 *        it may not make correct decision every time. Call this function to
 *        inform tracking engine that it gets lost based on external cues.
 *        Re-localization will be triggered after this is called.
 * \return success or not
 */
bool set_tracking_lost();
/**
 * \brief Inform tracking engine that the device is in an absolute status
 *        All the features that change location will be removed
 * \return success or not
 */
bool set_static(bool is_static);
/**
 * \brief Add the latest frame as keyframe
 * \param x A pointer for saving x. X axis is defined as right hand direction of the sensor.
 * \param y A pointer for saving y. Y axis is defined as the frontal direction of the sensor.
 * \param yaw A pointer for saving yaw value. The range of yaw is (-pi, pi].
 *            yaw is the clock-wise angle from the camera front axis.
 *            yaw = atan2(cam_z_in_W(0), cam_z_in_W(1))
 *            cam_z_in_W = W_R_Cl * Vector3f(0, 0, 1)
 * \return success or not
 */
bool add_latest_as_keyframe(float *x, float *y, float *yaw);
/*! The direction to send to the motor */
enum MotorDirection {
  Front = 0,
  Left = 1,
  Right = 2,
};
/**
 * \brief Get the direction for the shortest path toward the next previously visited location
 *        If a previous map is loaded, repeating calling this funtion will lead the robot
 *   	  to exactlly follow the previous path (e.g. a circle or from point A to point B)
 * \param motor_direction The direction to send to the motor
 * \return success or not
 */
bool get_direction_to_the_next_land_mark(MotorDirection* motor_direction);

/**
 * \brief This function that should be called by the live sensor for every stereo frame to
 *        pass image data into XP tracker
 * \param img_l left image
 * \param img_r right image
 * \param ts_100us timestamp in 100us (the exposure timestamp measured by the clock on sensor)
 * \param sys_time The chrono time point when the system receive the image data
 */
void image_data_callback(const cv::Mat& img_l,
                         const cv::Mat& img_r,
                         const float ts_100us,
                         const std::chrono::time_point<std::chrono::steady_clock>& sys_time);

/**
 * \brief This function that should be called by the live sensor for every IMU measurment to
 *        pass IMu data into XP tracker
 */
void imu_data_callback(const XPDRIVER::ImuData& imu_data);

/**
 * \brief Set the callback function that reports the image data rate and imu data rate,
 *        which will be used when calling draw_once.  This function only takes effect when
 *        running with a live sensor.
 */
typedef std::function<void(float* img_rate, float* imu_rate)> DataRateCallback;
bool set_data_rate_callback(const DataRateCallback& data_rate_callback);

/**
 * \brief The utility function that display the configuration of packages of current binary (libXP)
 *        as there are quite some functionalities that are enabled / disabled during compile time
 */
void print_XP_configuration();
}  // namespace XP_TRACKER
#endif  // __XP_TRACKER_H__  // NOLINT
