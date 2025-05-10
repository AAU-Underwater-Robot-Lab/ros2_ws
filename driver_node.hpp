#ifndef OCULUS_SONAR_DRIVER__DRIVER_NODE_HPP_
#define OCULUS_SONAR_DRIVER__DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <apl_msgs/msg/raw_data.hpp>
#include <memory>
#include <liboculus/oculus_client.hpp>

namespace oculus_sonar_driver
{

class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions & options);

private:
  void onData(const std::vector<uint8_t> & buffer);
  void setupParameters();
  rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_pub_;
  rclcpp::Publisher<apl_msgs::msg::RawData>::SharedPtr raw_pub_;
  std::shared_ptr<oculus::OculusClient> client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace oculus_sonar_driver

#endif  // OCULUS_SONAR_DRIVER__DRIVER_NODE_HPP_
