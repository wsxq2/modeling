#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import threading
import time

class PointCloudAccumulator(Node):
    """累积点云数据生成3D地图"""
    
    def __init__(self):
        super().__init__('pointcloud_accumulator')
        
        # 参数
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('max_points', 1000000)  # 最大点数限制
        self.declare_parameter('voxel_size', 0.05)     # 体素滤波大小
        self.declare_parameter('publish_rate', 1.0)    # 发布频率(Hz)
        
        self.map_frame = self.get_parameter('map_frame').value
        self.max_points = self.get_parameter('max_points').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅原始点云
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/cx/pointcloud',
            self.pointcloud_callback,
            10
        )
        
        # 发布累积的3D地图
        self.map_publisher = self.create_publisher(
            PointCloud2,
            '/accumulated_pointcloud_map',
            1
        )
        
        # 存储累积的点云数据（简化实现）
        self.accumulated_points = []
        self.lock = threading.Lock()
        
        # 定时发布
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_map)
        
        self.get_logger().info('PointCloud Accumulator started')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info(f'Max points: {self.max_points}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
    
    def pointcloud_callback(self, msg):
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 变换点云到地图坐标系
            transformed_cloud = do_transform_cloud(msg, transform)
            
            # 简化实现：存储变换后的点云消息
            with self.lock:
                self.accumulated_points.append(transformed_cloud)
                
                # 限制存储的点云数量
                if len(self.accumulated_points) > 100:  # 保留最近100帧
                    self.accumulated_points.pop(0)
                    
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {str(e)}')
    
    def publish_map(self):
        """发布累积的3D地图"""
        if not self.accumulated_points:
            return
            
        try:
            with self.lock:
                # 简化实现：发布最新的点云作为地图
                # 实际应用中应该合并所有点云并进行体素滤波
                if self.accumulated_points:
                    latest_cloud = self.accumulated_points[-1]
                    latest_cloud.header.frame_id = self.map_frame
                    latest_cloud.header.stamp = self.get_clock().now().to_msg()
                    
                    self.map_publisher.publish(latest_cloud)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to publish map: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()