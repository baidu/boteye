# This is an example script to perform stereo camera calibration.
# [IMPORTANT] record_path should be set
${MASTER_DIR}/bin/app_tracking
--navigation_folder=$record_path/navigation/ \
--navi_config=${MASTER_DIR}/XP/config/navigation_param.yaml \
--calib_file=$HOME/Boteye/calibration/calib.yaml \
--sensor_type=XP3
