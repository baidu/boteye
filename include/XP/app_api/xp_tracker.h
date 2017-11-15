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

#ifndef __XP_TRACKER_H__  // NOLINT
#define __XP_TRACKER_H__  // NOLINT
#include <XP/app_api/pose_packet.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <functional>

namespace XP_TRACKER {
/**
 * \brief Initialzie the LI sensor with LI driver. This has to be called before init_tracker
 * \brief If this fails, it is a driver issue.
 *        Currently, only supports LI sensor on Linux OS.  Will return false for other OS.
 * \param auto_gain Whether or not use auto gain and exposure control.
 * \param FPS Range [1 - 30]. 20 is typicall a good value. 30 may drop frames. 10 is not robust.
 * \return success or not
 */
bool init_sensor_LI(const std::string& sensor_file,
                    bool auto_gain = true);
/**
 * \brief Initialzie the XP sensor with XP driver. This has to be called before init_tracker
 * \brief If this fails, it is a driver issue.
 *        Currently, only supports XP sensor on Linux OS.  Will return false for other OS.
 * \param video_file file to the video device (e.g. /dev/video1)
 * \param imu_file file to the imu device (e.g. /dev/ttyUSB1)
 * \param auto_gain Whether or not use auto gain and exposure control.
 * \param imu_from_image Whether or not pull imu from image data.
 * \return success or not
 */
bool init_sensor_XP(const std::string& video_file,
                    const std::string& imu_file,
                    bool auto_gain = true,
                    bool imu_from_image = false);
/**
 * \brief Initialzie the XP sensor with XP driver. This has to be called before init_tracker
 * \brief If this fails, it is a driver issue.
 *        Currently, only supports XP sensor on Linux OS.  Will return false for other OS.
 * \param video_file file to the video device (e.g. /dev/video1)
 * \param imu_file file to the imu device (e.g. /dev/ttyUSB1)
 * \param imu_from_image Whether or not pull imu from image data.
 * \param auto_gain Whether or not use auto gain and exposure control.
 * \return success or not
 */
bool init_sensor_XP2(const std::string& video_file,
                     const std::string& imu_file,
                     bool auto_gain = true,
                     bool imu_from_image = false);
/**
* \brief Initialzie the XP sensor with opencv driver. This has to be called before init_tracker
* \brief This has to be run with xperception customized OpenCV.
*        Currently, only working with standard OpenCV.
* \param auto_gain Whether or not use auto gain and exposure control.
* \return success or not
*/
bool init_sensor_OCV(bool auto_gain = true);


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
 *        Currently only support LI/XP/XP2 sensors.
 * \param record_path record file path (the base folder)
 * \param cam_calib_path sensor calibration yaml file path (optional)
 *                       If provided, the calib file will also be saved to record_path
 * \return success or not
 */
std::string set_record_path(const std::string& record_path, const std::string& cam_calib_path = "");
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
 * \return success or not
 */
bool set_camera_view_canvas(cv::Mat* canvas);
/**
 * \brief Save draw canvas for depth image.
 *        If this is unset, the depth computation will be disabled (which saves ~100% cpu).
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 * \return success or not
 */
bool set_depth_view_canvas(cv::Mat* canvas);
/**
 * \brief Enable the path follower for robot control (EAI dashgo)
 * \note Can only enable path follower for either robot control or walking guide.
 * \return True if successful
 */
bool set_path_follower_robot();
/**
 * \brief Register callback functions for getting walk guide information
 * \param callback the call back function if a new walk guide message is ready
 */
bool set_path_follower_walk_callback(const XP_TRACKER::GuideMessageCallback& callback);

/**
 * \brief Save draw canvas for path follower control image.
 * \param canvas preallocated cv::Mat CV_8UC3 for drawing. Canvas can be a sub-matrix.
 * \return success or not
 */
bool set_path_follower_canvas(cv::Mat* canvas);
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
 * \brief Callback function for getting raw imgs
 * left img, right img, timestamp in seconds
 */
typedef std::function<void(const cv::Mat_<uchar>&,
                           const cv::Mat_<uchar>&, float)> StereoImagesCallback;
/**
 * \brief Register callback functions for getting raw imgs
 * \param callback the call back function if a new image pair is ready. This call back will
 *                 BLOCK the whole sensor image pulling thread, so callback has to be very
 *                 light. This function only works with LI sensor mode
 */
bool set_stereo_images_callback(const StereoImagesCallback& callback);
/**
 * \brief Get 2d pose based on mapper result
 * 				If no previous map is loaded, [0, 0] is defined as the initial position of the sensor.
 * 				If a previous map is loaded, [0, 0] and orientation is inherited from the previous map.
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
 * 				If no previous map is loaded, [0, 0, 0] is defined as the initial position.
 *        Z is up, X is right, Y is front.
 * 				If a previous map is loaded, positon and orientation is inherited from the previous map.
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
 * 				If no previous map is loaded, [0, 0, 0] is defined as the initial position.
 *        Z is up, X is right, Y is front.
 * 				If a previous map is loaded, positon and orientation is inherited from the previous map.
 * \param W_T_D_4x4 4x4 row matrix. It is the transformation between World and Device,
 *									following A_T_B definition. W_T_D_4x4 has to be preallocated.
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
}  // namespace XP_TRACKER
#endif  // __XP_TRACKER_H__  // NOLINT
