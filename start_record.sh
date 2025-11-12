set -e
. install/setup.bash
ros2 launch cartographer_config sensors.launch.py &
ros2 topic echo /imu_data --once
ros2 topic echo /cx/pointcloud --once
sleep 3
ros2 bag record /imu /cx/pointcloud