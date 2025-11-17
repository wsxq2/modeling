# modeling

本项目使用 3D 激光雷达和 IMU 实现 3D 建模，使用的算法是 cartographer。

## 目录结构

```
.vscode/
src/
├── cartographer_config/ # cartographer 配置文件，不是源码
├── lslidar/ # 激光雷达驱动的自定义配置文件和 launch 文件，以适应我们的需求
├── Lslidar_ROS2_driver/ # 镭神激光雷达C16官方驱动
├── ray_imu_ros2/ # 灵兮 IMU LX358 官方驱动
└── serial-ros2/ # IMU 驱动依赖的底层串口驱动
```