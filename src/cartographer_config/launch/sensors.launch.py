#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for starting sensors: IMU and LiDAR
    """
    
    # Declare launch arguments
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )
    
    pkg_share = FindPackageShare('cartographer_config').find('cartographer_config')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'my_robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}],
        output = 'screen'
        )

    # IMU Node
    imu_node = Node(
        package='ray_imu_ros2',
        executable='ray_imu_node',
        name='ray_imu_node',
        parameters=[{
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }],
        output='screen',
        respawn=True,
        respawn_delay=2
    )
    
    # LiDAR Launch File
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lslidar'),
                'launch',
                'lslidar_cx_launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        imu_frame_id_arg,
        robot_state_publisher_node,
        imu_node,
        lidar_launch
    ])


if __name__ == '__main__':
    generate_launch_description()