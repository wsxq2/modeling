. install/setup.bash
ros2 launch fdilink_ahrs ahrs_driver.launch.py &
ros2 launch lslidar lslidar_cx_launch.py &
sleep 3
ros2 bag record /imu /cx/pointcloud