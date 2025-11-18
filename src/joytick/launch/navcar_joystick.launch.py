#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev", 
            default_value="/dev/input/js0",
            description="Joystick device path",
        )
    )

    # Initialize Arguments
    joy_dev = LaunchConfiguration("joy_dev")

    # Joy node
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joystick",
        parameters=[
            {"device_name": joy_dev},
            {"deadzone": 0.1},
            {"autorepeat_rate": 10.0},
        ],
    )

    # Bag recorder control node  
    bag_control_node = Node(
        package="joytick",
        executable="bag_recorder_control",
        name="bag_recorder_control_node",
        parameters=[
            {"x_button_index": 2},  # X button index from jstest
            {"topics": ["/imu", "/cx/pointcloud"]},
        ],
    )

    nodes = [
        joy_node,
        bag_control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
