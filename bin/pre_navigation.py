'''
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

Sample run:
python pre_navigation.py
--record_path=
--build_folder=

'''
import os
import argparse
import glob
import subprocess
import shutil

def main():
    bow_proto = os.environ['HOME'] + '/XP_release/3rdparty_lib_lean/BOW.proto'

    parser = argparse.ArgumentParser()
    parser.add_argument('--record_path', type=str, default=None,
        help='Path of the recorded data')
    parser.add_argument('--build_folder', type=str, default=None, help=
        'Path to custom built binaries to override the default bin locations.')
    parser.add_argument('--waypoints_config', type=str, default=None, help=
        'Path of way points config file.')
    parser.add_argument('--priori_traj_file', type=str, default=None, help=
    'Path of priori waypoints csv file')
    parser.add_argument('--add_config_waypoints', type=bool, default=False,
                        help='Add waypoints from config file.')
    parser.add_argument('--customize_trajectory', type=bool, default=False,
                        help='Enable waypoints customization.')
    parser.add_argument('--use_priori_traj', type=bool, default=False,
                        help='Use priori traj csv instead of map_pb to init frames.')
    parser.add_argument('--connect_adjacent_id', type=bool, default=True,
                        help='Generate connectivity based on waypoints adjacency.')
    args = parser.parse_args()

    # In releases, binaries are in $MASTER_DIR/bin
    bin_folder = os.environ['MASTER_DIR'] + '/bin/'
    binaries = [
        bin_folder + '/dump_map_info',
        bin_folder + '/waypoints_generate_map',
        bin_folder + '/waypoint_graph']
    if not os.path.isdir(bin_folder):
        # In built binaries are assumed to be in
        bin_folder = os.environ['MASTER_DIR'] + '/build/'
        binaries = [
            bin_folder + '/pc_apps/utils/dump_map_info',
            bin_folder + '/pc_apps/navigation/waypoints_generate_map',
            bin_folder + '/pc_apps/navigation/waypoint_graph']
    if args.build_folder:
        build_folder = args.build_folder
        binaries = [
            build_folder + '/pc_apps/utils/dump_map_info',
            build_folder + '/pc_apps/navigation/waypoints_generate_map',
            build_folder + '/pc_apps/navigation/waypoint_graph']

    for binary in binaries:
        if not os.path.isfile(binary):
            raise ValueError(binary + ' does not exist!')

    record_path = args.record_path
    navigation_path = record_path + '/navigation/'
    if not os.path.isdir(navigation_path):
        os.mkdir(navigation_path)

    shutil.copyfile(record_path + '/live.pb', navigation_path + '/navi.pb')

    # mkdir -p ${NAVIGATION_PATH}
    # ${BUILD_DIRECTORY}/pc_apps/utils/dump_map_info \
    # -map_pb  ${RECORD_PATH}/tracking.pb > ${NAVIGATION_PATH}/waypoints.csv
    customize_trajectory = ''
    waypoints_config = ''
    add_config_waypoints = ''
    priori_traj_file = ''
    use_priori_traj = ''
    if args.waypoints_config:
        waypoints_config = '-waypoints_config ' + args.waypoints_config
    if args.add_config_waypoints:
        add_config_waypoints = '-add_config_waypoints'
    if args.priori_traj_file:
        priori_traj_file = '-priori_traj_file' + args.priori_traj_file
    if args.use_priori_traj:
        use_priori_traj = '-use_priori_traj'
    if args.customize_trajectory:
        customize_trajectory = '-customize_trajectory'
    command = ' '.join([
        binaries[0],
        waypoints_config,
        add_config_waypoints,
        priori_traj_file,
        use_priori_traj,
        customize_trajectory,
        '-map_pb ' + record_path + '/live.pb',
        '>', navigation_path + '/waypoints.csv'])
    print(command)
    subprocess.call(command, shell = True)

    # # Waypoints_generate_map: csv -> map
    # ${BUILD_DIRECTORY}/pc_apps/navigation/waypoints_generate_map \
    # -waypoints_file ${NAVIGATION_PATH}/waypoints.csv \
    # -output_folder=${NAVIGATION_PATH}

    command = ' '.join([
        binaries[1],
        '-waypoints_file ' + navigation_path + '/waypoints.csv',
        '-output_folder ' + navigation_path])
    print(command)
    subprocess.call(command, shell = True)

    # # Generate waypoints connectivity graph
    # ${BUILD_DIRECTORY}/pc_apps/navigation/waypoint_graph \
    # -waypoints_file ${NAVIGATION_PATH}/waypoints.csv \
    # -waypoints_only_mode \
    # -map_occupancy_file ${NAVIGATION_PATH}/map_occupancy.png \
    # -map_occupancy_specs_file ${NAVIGATION_PATH}/map_occupancy.yml \
    # -waypoint_graph_output ${NAVIGATION_PATH}/waypoints_graph.txt

    connect_adjacent_id = '-connect_adjacent_id'
    if not args.connect_adjacent_id:
        connect_adjacent_id = ''
    command = ' '.join([
        binaries[2],
        '-waypoints_file ' + navigation_path + '/waypoints.csv',
        '-waypoints_only_mode',
        connect_adjacent_id,
        '-map_occupancy_file ' + navigation_path + '/map_occupancy.png',
        '-map_occupancy_specs_file ' + navigation_path + '/map_occupancy.yml',
        '-waypoint_graph_output ' + navigation_path + '/waypoints_graph.txt'])
    print(command)
    subprocess.call(command, shell = True)

if __name__ == "__main__":
    main()
