#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # IMU Node
    imu_node = Node(
        package='ray_imu_ros2',
        executable='ray_imu_node',
        name='ray_imu_node',
        parameters=[{
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }],
        output='screen',
        # respawn=True,
        # respawn_delay=2
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
        imu_node,
        lidar_launch
    ])


if __name__ == '__main__':
    generate_launch_description()