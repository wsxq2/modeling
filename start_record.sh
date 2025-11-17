set -e
. install/setup.bash
# ros2 launch cartographer_config sensors.launch.py &
ros2 run ray_imu_ros2 ray_imu_node --ros-args -p imu_frame_id:=imu_link &
ros2 launch lslidar lslidar_cx_launch.py &
ros2 topic echo /imu_data --once
ros2 topic echo /cx/pointcloud --once
sleep 3
ros2 bag record /imu /cx/pointcloud