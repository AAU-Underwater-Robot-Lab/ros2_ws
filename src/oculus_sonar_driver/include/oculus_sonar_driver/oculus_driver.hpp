#ifndef OCULUS_SONAR_DRIVER__OCULUS_DRIVER_HPP_
#define OCULUS_SONAR_DRIVER__OCULUS_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "oculus_sonar_driver/publishing_data_rx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/StatusRx.h"
#include "liboculus/SonarConfiguration.h"
#include "liboculus/SimplePingResult.h"

namespace oculus_sonar_driver {

class OculusDriver : public rclcpp::Node {
 public:
  explicit OculusDriver(const rclcpp::NodeOptions & options);
  void init_communication();

 private:
  PublishingDataRx data_rx_;
  liboculus::StatusRx status_rx_;
  liboculus::IoServiceThread io_service_;
  rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  double gain_;
};

} // namespace oculus_sonar_driver

#endif  // OCULUS_SONAR_DRIVER__OCULUS_DRIVER_HPP_