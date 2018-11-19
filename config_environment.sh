#!/bin/bash
echo "Configure environment variables for XP"
export MASTER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Set MASTER_DIR="${MASTER_DIR}

# Check if 3rdparty_lib_lean exists
thirdparty_lib_dir=~/XP_release/3rdparty_lib_lean/lib
if [ ! -d $thirdparty_lib_dir ]; then
  printf "\033[0;31m Cannot find 3rdparty_lib_lean for XP! \n\033[0m"
  return
fi

if [ "$(uname -s)" = "Linux" ]; then
  echo "OS:Linux detected"
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${MASTER_DIR}/lib_$(uname -m):${thirdparty_lib_dir}
  echo "LD_LIBRARY_PATH="${LD_LIBRARY_PATH}
elif [ "$(uname -s)" = "Darwin" ]; then
  echo "OS:Darwin detected"
  export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:${MASTER_DIR}/lib_x86_64
  echo "DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH}
else
  echo "Unsupported platform!"
  return
fi
printf "\033[0;32m config environment ok! \n\033[0m"
