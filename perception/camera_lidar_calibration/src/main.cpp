#include "camera_lidar_calibration/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<camera_lidar_calibration::ExtrinsicCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
