#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "mi0802_senxor_driver/mi0802_device.hpp"

namespace mi0802_senxor_driver
{

std::vector<float> orient_frame(
  const ThermalFrame & frame, bool flip_horizontal, bool flip_vertical, bool rotate_180);

sensor_msgs::msg::Image make_image(
  const std::vector<float> & temperatures,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id);

class ReconnectWaiter
{
public:
  bool wait_for(std::chrono::duration<double> delay, const std::atomic_bool & stop_requested);
  void notify();

private:
  std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace mi0802_senxor_driver
