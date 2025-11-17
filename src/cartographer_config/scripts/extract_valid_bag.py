#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosbag2_py import SequentialWriter
from rosidl_runtime_py.utilities import get_message
import argparse
from datetime import datetime


class BagExtractor(Node):
    def __init__(self):
        super().__init__('bag_extractor')
        
    def analyze_bag_timerange(self, input_bag_path, imu_topic='/imu', lidar_topic='/cx/pointcloud'):
        """
        分析bag文件，找到同时有IMU和激光雷达数据的时间范围
        """
        storage_options = StorageOptions(uri=input_bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        # 获取话题信息
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        print(f"分析bag文件: {input_bag_path}")
        print(f"寻找话题: IMU={imu_topic}, LiDAR={lidar_topic}")
        
        # 收集时间戳
        imu_timestamps = []
        lidar_timestamps = []
        
        message_count = 0
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            message_count += 1
            
            if message_count % 1000 == 0:
                print(f"已处理消息: {message_count}")
            
            if topic == imu_topic:
                imu_timestamps.append(timestamp)
            elif topic == lidar_topic:
                lidar_timestamps.append(timestamp)
        
        # reader.close()  # 在某些ROS2版本中不需要显式关闭
        
        print(f"总消息数: {message_count}")
        print(f"IMU消息数: {len(imu_timestamps)}")
        print(f"LiDAR消息数: {len(lidar_timestamps)}")
        
        if not imu_timestamps or not lidar_timestamps:
            print("错误: 缺少IMU或LiDAR数据!")
            return None, None
        
        # 找到有效时间范围
        imu_start = min(imu_timestamps)
        imu_end = max(imu_timestamps)
        lidar_start = min(lidar_timestamps)
        lidar_end = max(lidar_timestamps)
        
        # 有效范围是两者的交集
        valid_start = max(imu_start, lidar_start)
        valid_end = min(imu_end, lidar_end)
        
        print(f"\n时间范围分析:")
        print(f"IMU时间范围: {self.timestamp_to_str(imu_start)} - {self.timestamp_to_str(imu_end)}")
        print(f"LiDAR时间范围: {self.timestamp_to_str(lidar_start)} - {self.timestamp_to_str(lidar_end)}")
        print(f"有效时间范围: {self.timestamp_to_str(valid_start)} - {self.timestamp_to_str(valid_end)}")
        
        duration = (valid_end - valid_start) / 1e9
        print(f"有效持续时间: {duration:.2f} 秒")
        
        return valid_start, valid_end
    
    def timestamp_to_str(self, timestamp_ns):
        """将纳秒时间戳转换为可读字符串"""
        timestamp_s = timestamp_ns / 1e9
        dt = datetime.fromtimestamp(timestamp_s)
        return dt.strftime('%H:%M:%S.%f')[:-3]  # 显示到毫秒
    
    def extract_valid_portion(self, input_bag_path, output_bag_path, start_time, end_time, 
                            topics_to_extract=None):
        """
        提取指定时间范围内的bag数据
        """
        if topics_to_extract is None:
            topics_to_extract = ['/imu', '/cx/pointcloud', '/imu_data']
        
        # 设置读取器
        storage_options_in = StorageOptions(uri=input_bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        
        reader = SequentialReader()
        reader.open(storage_options_in, converter_options)
        
        # 设置写入器
        storage_options_out = StorageOptions(uri=output_bag_path, storage_id='sqlite3')
        writer = SequentialWriter()
        writer.open(storage_options_out, converter_options)
        
        # 获取话题信息并创建话题
        topic_types = reader.get_all_topics_and_types()
        for topic_info in topic_types:
            if topic_info.name in topics_to_extract:
                writer.create_topic(topic_info)
                print(f"创建话题: {topic_info.name} ({topic_info.type})")
        
        # 提取数据
        extracted_count = 0
        total_count = 0
        
        print(f"\n开始提取数据...")
        print(f"时间范围: {self.timestamp_to_str(start_time)} - {self.timestamp_to_str(end_time)}")
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            total_count += 1
            
            # 检查是否在有效时间范围内
            if start_time <= timestamp <= end_time and topic in topics_to_extract:
                writer.write(topic, data, timestamp)
                extracted_count += 1
                
                if extracted_count % 100 == 0:
                    progress = (timestamp - start_time) / (end_time - start_time) * 100
                    print(f"提取进度: {progress:.1f}% (已提取 {extracted_count} 条消息)")
        
        # reader.close()  # API兼容性问题
        # writer.close()  # API兼容性问题
        
        print(f"\n提取完成!")
        print(f"总消息数: {total_count}")
        print(f"提取消息数: {extracted_count}")
        print(f"输出文件: {output_bag_path}")
        
        return extracted_count

def main():
    parser = argparse.ArgumentParser(description='提取bag文件中同时包含IMU和LiDAR数据的有效部分')
    parser.add_argument('input_bag', help='输入bag文件路径')
    parser.add_argument('-o', '--output', help='输出bag文件路径', 
                       default=None)
    parser.add_argument('--imu-topic', default='/imu', 
                       help='IMU话题名称 (默认: /imu)')
    parser.add_argument('--lidar-topic', default='/cx/pointcloud', 
                       help='LiDAR话题名称 (默认: /cx/pointcloud)')
    parser.add_argument('--topics', nargs='+', 
                       default=['/imu', '/cx/pointcloud', '/imu_data'],
                       help='要提取的话题列表')
    parser.add_argument('--analyze-only', action='store_true',
                       help='仅分析时间范围，不提取数据')
    
    args = parser.parse_args()
    
    # 设置默认输出路径
    if args.output is None:
        input_name = os.path.splitext(os.path.basename(args.input_bag))[0]
        args.output = f"{input_name}_valid_portion"
    
    # 初始化ROS2
    rclpy.init()
    extractor = BagExtractor()
    
    try:
        # 分析时间范围
        start_time, end_time = extractor.analyze_bag_timerange(
            args.input_bag, args.imu_topic, args.lidar_topic)
        
        if start_time is None or end_time is None:
            print("无法找到有效的时间范围!")
            return 1
        
        if args.analyze_only:
            print("仅分析模式，不提取数据")
            return 0
        
        # 提取有效部分
        extracted_count = extractor.extract_valid_portion(
            args.input_bag, args.output, start_time, end_time, args.topics)
        
        if extracted_count > 0:
            print(f"\n成功提取 {extracted_count} 条消息到: {args.output}")
            return 0
        else:
            print("没有提取到任何数据!")
            return 1
            
    except Exception as e:
        print(f"错误: {e}")
        return 1
    finally:
        extractor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())