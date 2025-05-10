#include "oculus_sonar_driver/driver_node.hpp"
#include <chrono>

namespace oculus_sonar_driver
{

DriverNode::DriverNode(const rclcpp::NodeOptions & options)
: Node("oculus_sonar_driver", options)
{
  setupParameters();

  sonar_pub_ = create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
    "sonar/image/projected", 10);
  raw_pub_ = create_publisher<apl_msgs::msg::RawData>("sonar/raw", 10);

  client_ = std::make_shared<oculus::OculusClient>(
    get_parameter("host").as_string(),
    get_parameter("port").as_int());

  client_->setCallback([this](const std::vector<uint8_t> & buffer) {
    onData(buffer);
  });

  client_->start();
}

void DriverNode::setupParameters()
{
  this->declare_parameter<std::string>("host", "192.168.2.200");
  this->declare_parameter<int>("port", 9090);
}

void DriverNode::onData(const std::vector<uint8_t> & buffer)
{
  auto raw_msg = apl_msgs::msg::RawData();
  raw_msg.data = buffer;
  raw_pub_->publish(raw_msg);

  auto img_msg = marine_acoustic_msgs::msg::ProjectedSonarImage();
  client_->parseProjectedImage(buffer, img_msg);
  sonar_pub_->publish(img_msg);
}

}  // namespace oculus_sonar_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(oculus_sonar_driver::DriverNode)
