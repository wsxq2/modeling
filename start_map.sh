set -u

bag_filename=${1}

set +u

. install/setup.bash
ros2 launch cartographer_config offline_my_robot.launch.py \
     bag_filenames:=${bag_filename} 

ros2 launch cartographer_config assets_writer_my_robot.launch.py \
     bag_filenames:=${bag_filename} \
     pose_graph_filename:=${bag_filename}.pbstream \