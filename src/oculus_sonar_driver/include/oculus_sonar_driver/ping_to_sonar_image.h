#pragma once

#include <vector>
#include "liboculus/Constants.h"
#include "liboculus/SimplePingResult.h"
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

namespace oculus_sonar_driver {

// Packs an acoustic_msgs::ProjectedSonarImage from the contents of a SimplePingResult
template <typename PingT>
void convert_ping_to_sonar_image(const liboculus::SimplePingResult<PingT> &ping,
                                 marine_acoustic_msgs::msg::ProjectedSonarImage &sonar_image) {
  sonar_image.ping_info.frequency = ping.ping()->frequency;
  // sonar_image.ping_info.sound_speed = ... // Add if sound speed available
  sonar_image.ping_info.range = ping.ping()->range;
  sonar_image.ping_info.ping_number = ping.ping()->pingId;

  sonar_image.image.data.assign(ping.imageData().begin(), ping.imageData().end());
  sonar_image.image.height = ping.ping()->nRows;
  sonar_image.image.width = ping.ping()->nBeams;

  sonar_image.image.encoding = "mono16";
  sonar_image.image.step = sonar_image.image.width * sizeof(uint16_t);
}

}  // namespace oculus_sonar_driver
