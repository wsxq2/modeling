#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Bringup launch file that includes sensors and joystick functionality
    """
    
    # Declare launch arguments
    declared_arguments = []
    
    # Joy device argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev", 
            default_value="/dev/input/js0",
            description="Joystick device path",
        )
    )
    
    # IMU frame ID argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='imu_link',
            description='Frame ID for IMU data'
        )
    )
    
    # X button index argument for bag recording
    declared_arguments.append(
        DeclareLaunchArgument(
            'x_button_index',
            default_value='2',
            description='X button index for PS3 controller (usually 2)'
        )
    )
    
    # Recording topics argument
    declared_arguments.append(
        DeclareLaunchArgument(
            'record_topics',
            default_value='/imu /cx/pointcloud',
            description='Space-separated list of topics to record'
        )
    )
    
    # Include sensors launch file from cartographer_config
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cartographer_config'),
                'launch',
                'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }.items()
    )
    
    # Include joystick launch file
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('joytick'),  # Note: package name is 'joytick' as seen in navcar_joystick.launch.py
                'launch',
                'navcar_joystick.launch.py'
            ])
        ]),
        launch_arguments={
            'joy_dev': LaunchConfiguration('joy_dev')
        }.items()
    )
    
    # Return launch description with all components
    return LaunchDescription(
        declared_arguments + [
            sensors_launch,
            joystick_launch,
        ]
    )


if __name__ == '__main__':
    generate_launch_description()
