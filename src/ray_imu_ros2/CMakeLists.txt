cmake_minimum_required(VERSION 3.8)
project(ray_imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# 声明消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IMUData.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_executable(ray_imu_node src/ray_imu_node.cpp)
ament_target_dependencies(ray_imu_node
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  serial
)

rosidl_target_interfaces(ray_imu_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(imu_printer_node src/imu_printer_node.cpp)
ament_target_dependencies(imu_printer_node rclcpp std_msgs)
rosidl_target_interfaces(imu_printer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  ray_imu_node
  imu_printer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
