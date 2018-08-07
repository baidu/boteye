# This is an example script to perform stereo camera calibration.
# [IMPORTANT] The square_size has to be precisely measured.
${MASTER_DIR}/build/pc_apps/cam_calibration/\
./cam_calibration \
--square_size=0.03 \
--save_calib_yaml=$HOME/Boteye/calibration/calib.yaml \
--show_reproj \
--record_path=$HOME/Boteye/calibration/images/