# This is an example script to perform SLAM data recording.
# This script takes one input argument to specify the record_path. If not provided, this script will attempt to use the environment variable $record_path as the record_path.
if [ "$#" -eq 0 ]; then
  while true; do
    read -p "Record folder is not specified. Record data in default folder: $HOME/Boteye/data/seq? (y/n)" yn
    case $yn in
        [Yy]* ) real_record_folder=$HOME/Boteye/data/seq; mkdir -vp $real_record_folder; break;;
        [Nn]* ) echo "stopped"; exit 0;;
        * ) echo "Please answer yes or no.";;
    esac
  done
else
  if [ "$#" -ne 1 ]; then
    echo "ERROR Too many input arguments. Please only input the record folder.";
    exit 0;
  else
    real_record_folder=$1;
    mkdir -vp $real_record_folder;
    if [ "$(ls -A ${real_record_folder})" ]; then
      while true; do
        read -p "The folder ${real_record_folder} contains other files. Do you still want to record data in it? (y/n)" yn
        case $yn in
          [Yy]* ) break;;
          [Nn]* ) echo "stopped"; exit 0;;
          * ) echo "Please answer yes or no.";;
        esac
      done
    fi
  fi
fi

${MASTER_DIR}/build/pc_apps/app_tracking/\
./app_tracking \
--record_path=$real_record_folder \
--sensor_type=XP3 \
--calib_file=$HOME/Boteye/calibration/calib.yaml \
