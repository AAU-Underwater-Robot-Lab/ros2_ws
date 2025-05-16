#pragma once

#include <vector>
#include <functional>
#include "liboculus/DataRx.h"

namespace oculus_sonar_driver {

class PublishingDataRx : public liboculus::DataRx {
public:
  using DataCallback = std::function<void(const std::vector<uint8_t>&, uint8_t)>;

  explicit PublishingDataRx(const liboculus::IoServiceThread::IoContextPtr &iosrv)
    : DataRx(iosrv), count_(0) {}

  ~PublishingDataRx() {}

  void set_data_callback(DataCallback cb) {
    data_callback_ = cb;
  }

  void haveWritten(const std::vector<uint8_t> &bytes) override {
    publish_data(bytes, 1);  // Assume 1 = DATA_OUT
  }

  void haveRead(const std::vector<uint8_t> &bytes) override {
    publish_data(bytes, 0);  // Assume 0 = DATA_IN
  }

private:
  void publish_data(const std::vector<uint8_t> &bytes, uint8_t direction) {
    if (!bytes.empty() && data_callback_) {
      data_callback_(bytes, direction);
    }
    ++count_;
  }

  DataCallback data_callback_;
  uint32_t count_;
};

}  // namespace oculus_sonar_driver
