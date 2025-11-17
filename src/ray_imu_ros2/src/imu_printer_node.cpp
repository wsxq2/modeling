#include "rclcpp/rclcpp.hpp"
#include "ray_imu_ros2/msg/imu_data.hpp"
#include <sstream>
#include <iomanip>

void imu_callback(const ray_imu_ros2::msg::IMUData::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("imu_printer_node"),
            "[IMU]\n"
            "Ax=%.10f\n"
            "Ay=%.10f\n"
            "Az=%.10f\n"
            "Gx=%.10f\n"
            "Gy=%.10f\n"
            "Gz=%.10f\n"
            "Roll=%.10f\n"
            "Pitch=%.10f\n"
            "Yaw=%.10f\n"
            "delta_ms=%.3f\n"
            "protocol=%s\n"
            "hex=%s",
            msg->ax_g,
            msg->ay_g,
            msg->az_g,
            msg->gx_dps,
            msg->gy_dps,
            msg->gz_dps,
            msg->roll,
            msg->pitch,
            msg->yaw,
            msg->delta_ms,
            msg->protocol.c_str(),
            msg->hex.c_str());

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("imu_printer_node");

    auto sub = node->create_subscription<ray_imu_ros2::msg::IMUData>(
        "imu_data",
        10,
        imu_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
