#include "oculus_sonar_driver/oculus_driver.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "liboculus/Constants.h"

namespace oculus_sonar_driver {

using std::placeholders::_1;

OculusDriver::OculusDriver(const rclcpp::NodeOptions & options)
  : Node("oculus_driver", options),
    data_rx_(io_service_.context()),
    status_rx_(io_service_.context()) {

  this->declare_parameter<double>("gain", 0.5);
  gain_ = this->get_parameter("gain").as_double();

  sonar_pub_ = this->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>("sonar_image", 10);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params) {
      for (const auto &param : params) {
        if (param.get_name() == "gain") {
          gain_ = param.as_double();
          RCLCPP_INFO(this->get_logger(), "Updated gain to: %f", gain_);
        }
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  init_communication();
}

void OculusDriver::init_communication() {
  data_rx_.set_data_callback(
    [this](const std::vector<uint8_t>& bytes, uint8_t direction) {
      RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes direction %u", bytes.size(), direction);
    });

  data_rx_.set_ping_callback(std::bind(&OculusDriver::handle_ping, this, _1));

  if (!data_rx_.open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open sonar data receiver.");
    return;
  }

  io_service_.start();
}

void OculusDriver::handle_ping(const std::shared_ptr<liboculus::SimplePingResult> &ping) {
  if (!ping) return;

  auto msg = std::make_unique<marine_acoustic_msgs::msg::ProjectedSonarImage>();
  convert_ping_to_sonar_image(*ping, *msg);
  sonar_pub_->publish(std::move(msg));
}

}  // namespace oculus_sonar_driver

RCLCPP_COMPONENTS_REGISTER_NODE(oculus_sonar_driver::OculusDriver)
