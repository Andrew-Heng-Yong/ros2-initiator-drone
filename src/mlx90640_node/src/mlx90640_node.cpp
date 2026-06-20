#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/temperature.hpp"

extern "C" {
#include "MLX90640_API.h"
#include "mlx90640_node/mlx90640_i2c_linux.h"
}

class Mlx90640Node final : public rclcpp::Node {
 public:
  Mlx90640Node() : Node("mlx90640_node") {
    device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    address_ = declare_parameter<int>("i2c_address", 0x33);
    refresh_rate_ = declare_parameter<int>("refresh_rate", 4);
    emissivity_ = declare_parameter<double>("emissivity", 0.95);
    frame_id_ = declare_parameter<std::string>("frame_id", "mlx90640_optical_frame");
    topic_ = declare_parameter<std::string>("topic", "thermal/image_raw");
    if (address_ < 0 || address_ > 127 || refresh_rate_ < 0 || refresh_rate_ > 7 ||
        emissivity_ <= 0.0 || emissivity_ > 1.0) {
      throw std::invalid_argument("i2c_address must be 0..127, refresh_rate 0..7, emissivity (0, 1]");
    }
    MLX90640_I2CSetDevice(device_.c_str());
    std::array<uint16_t, MLX90640_EEPROM_DUMP_NUM> eeprom{};
    int error = MLX90640_DumpEE(address_, eeprom.data());
    if (error != MLX90640_NO_ERROR) throw std::runtime_error("could not read MLX90640 EEPROM: " + std::to_string(error));
    error = MLX90640_ExtractParameters(eeprom.data(), &parameters_);
    if (error != MLX90640_NO_ERROR) throw std::runtime_error("invalid MLX90640 calibration data: " + std::to_string(error));
    if ((error = MLX90640_SetRefreshRate(address_, refresh_rate_)) != MLX90640_NO_ERROR)
      throw std::runtime_error("could not set MLX90640 refresh rate: " + std::to_string(error));

    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
    ambient_publisher_ = create_publisher<sensor_msgs::msg::Temperature>("thermal/ambient", rclcpp::SensorDataQoS());
    // A full 32x24 image consists of two subpages. Polling faster than the configured rate
    // prevents the reference API's data-ready loop from delaying unrelated ROS callbacks.
    const auto period = std::chrono::duration<double>(1.0 / refresh_hz(refresh_rate_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&Mlx90640Node::publish_frame, this));
    RCLCPP_INFO(get_logger(), "MLX90640 on %s at 0x%02X; publishing %s", device_.c_str(), address_, topic_.c_str());
  }

 private:
  static double refresh_hz(int code) { return 0.5 * (1 << code); }
  void publish_frame() {
    std::array<uint16_t, 834> frame{};
    const int error = MLX90640_GetFrameData(address_, frame.data());
    if (error != MLX90640_NO_ERROR) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "MLX90640 frame read failed: %d", error);
      return;
    }
    std::array<float, MLX90640_PIXEL_NUM> subpage_temperatures{};
    const float ambient = MLX90640_GetTa(frame.data(), &parameters_);
    MLX90640_CalculateTo(frame.data(), &parameters_, static_cast<float>(emissivity_), ambient - 8.0F, subpage_temperatures.data());
    const int subpage = MLX90640_GetSubPageNumber(frame.data());
    const bool chess_mode = (frame[832] & MLX90640_CTRL_MEAS_MODE_MASK) != 0;
    for (int pixel = 0; pixel < MLX90640_PIXEL_NUM; ++pixel) {
      const int row_pattern = (pixel / 32) % 2;
      const int pattern = chess_mode ? row_pattern ^ (pixel % 2) : row_pattern;
      if (pattern == subpage) temperature_image_[pixel] = subpage_temperatures[pixel];
    }
    received_subpage_[subpage] = true;
    if (!received_subpage_[0] || !received_subpage_[1]) return;
    MLX90640_BadPixelsCorrection(parameters_.brokenPixels, temperature_image_.data(), 1, &parameters_);
    MLX90640_BadPixelsCorrection(parameters_.outlierPixels, temperature_image_.data(), 1, &parameters_);

    sensor_msgs::msg::Image image;
    image.header.stamp = now(); image.header.frame_id = frame_id_;
    image.height = MLX90640_LINE_NUM; image.width = MLX90640_COLUMN_NUM;
    image.encoding = "32FC1"; image.is_bigendian = false; image.step = image.width * sizeof(float);
    image.data.resize(temperature_image_.size() * sizeof(float));
    std::memcpy(image.data.data(), temperature_image_.data(), image.data.size());
    image_publisher_->publish(image);
    sensor_msgs::msg::Temperature ambient_message;
    ambient_message.header = image.header; ambient_message.temperature = ambient; ambient_message.variance = 0.0;
    ambient_publisher_->publish(ambient_message);
  }
  std::string device_, frame_id_, topic_; int address_, refresh_rate_; double emissivity_;
  paramsMLX90640 parameters_{};
  std::array<float, MLX90640_PIXEL_NUM> temperature_image_{};
  std::array<bool, 2> received_subpage_{{false, false}};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr ambient_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Mlx90640Node>()); }
  catch (const std::exception &error) { RCLCPP_FATAL(rclcpp::get_logger("mlx90640_node"), "%s", error.what()); }
  rclcpp::shutdown(); return 0;
}
