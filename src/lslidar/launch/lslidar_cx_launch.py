import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    driver_config = os.path.join(get_package_share_directory('lslidar'),'config','lslidar_c16.yaml')
    rviz_config = os.path.join(get_package_share_directory('lslidar'),'rviz','lslidar_cx.rviz')

    driver_node = LifecycleNode(package='lslidar_driver',
                                executable='lslidar_driver_node',
                                name='lslidar_driver_node',
                                namespace='cx', # 与对应yaml文件中命名空间一致
                                parameters=[driver_config],
                                output='screen'
                                )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        driver_node,
        # rviz_node
    ])
