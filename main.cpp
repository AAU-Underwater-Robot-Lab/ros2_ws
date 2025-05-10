#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<oculus_sonar_driver::DriverNode>(options));
  rclcpp::shutdown();
  return 0;
}
