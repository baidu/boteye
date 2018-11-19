# This is an example script to perform robot navigation.
# This script takes one input argument to specify the navigation_folder, which should contain the essential files for the navigation task. If not provided, this script will try to use $record_path/navigation as the navigation_folder.
if [ "$#" -eq 0 ]; then
  if [ -z "$record_path" ]; then
    echo "ERROR Neither navigation folder nor \$record_path is specified. Please specify one of them."
    exit 0;
  else 
    if [ ! -d "$record_path/navigation/" ] || [ -z "$(ls -A $record_path/navigation/)" ]; then
      echo "Navigation folder is not specified and the folder \$record_path/navigaiton/: $record_path/navigation/ is empty."
      exit 0;
    else
      while true; do
        read -p "Navigation folder is not specified. Use \$record_path/navigation/: $record_path/navigation/ as navigation folder? (y/n)" yn
        case $yn in
          [Yy]* ) navigation_folder=$record_path/navigation/; break;;
          [Nn]* ) echo "stopped"; exit 0;;
          * ) echo "Please answer yes or no.";;
        esac
      done
    fi
  fi
else
  if [ "$#" -ne 1 ]; then
    echo "ERROR Too many input arguments. Please only input the navigation folder.";
    exit 0;
  else
    if [ ! -d "$1" ] || [ -z "$(ls -A $1)" ]; then
      echo "ERROR Specified navigation folder: $1 is empty.";
      exit 0;
    else
      navigation_folder=$1
    fi
  fi
fi

${MASTER_DIR}/build/pc_apps/app_tracking/\
./app_tracking \
--navigation_folder=$navigation_folder \
--navi_config=${MASTER_DIR}/XP/config/navigation_param.yaml \
--calib_file=$HOME/Boteye/calibration/calib.yaml \
--sensor_type=XP3 \