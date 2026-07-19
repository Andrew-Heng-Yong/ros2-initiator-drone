#include "mi0802_senxor_driver/frame_utils.hpp"

#include <cstring>
#include <stdexcept>

namespace mi0802_senxor_driver
{

std::vector<float> orient_frame(
  const ThermalFrame & frame, bool flip_horizontal, bool flip_vertical, bool rotate_180)
{
  const bool reverse_x = flip_horizontal != rotate_180;
  const bool reverse_y = flip_vertical != rotate_180;
  std::vector<float> output(kPixelCount);
  for (std::size_t y = 0; y < kFrameHeight; ++y) {
    for (std::size_t x = 0; x < kFrameWidth; ++x) {
      const auto source_x = reverse_x ? kFrameWidth - 1 - x : x;
      const auto source_y = reverse_y ? kFrameHeight - 1 - y : y;
      output[y * kFrameWidth + x] = frame.temperatures[source_y * kFrameWidth + source_x];
    }
  }
  return output;
}

sensor_msgs::msg::Image make_image(
  const std::vector<float> & temperatures,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  if (temperatures.size() != kPixelCount) {
    throw std::invalid_argument("MI0802 image requires exactly 80 * 62 temperatures");
  }
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = frame_id;
  image.height = static_cast<std::uint32_t>(kFrameHeight);
  image.width = static_cast<std::uint32_t>(kFrameWidth);
  image.encoding = "32FC1";
  image.is_bigendian = false;
  image.step = static_cast<std::uint32_t>(kFrameWidth * sizeof(float));
  image.data.resize(temperatures.size() * sizeof(float));
  std::memcpy(image.data.data(), temperatures.data(), image.data.size());
  return image;
}

bool ReconnectWaiter::wait_for(
  std::chrono::duration<double> delay, const std::atomic_bool & stop_requested)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_.wait_for(lock, delay, [&stop_requested]() {return stop_requested.load();});
}

void ReconnectWaiter::notify()
{
  condition_.notify_all();
}

}  // namespace mi0802_senxor_driver
