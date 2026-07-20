#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "mi0802_senxor_driver/frame_utils.hpp"
#include "mi0802_senxor_driver/mi0802_device.hpp"

namespace mi0802_senxor_driver
{

class Mi0802SenxorNode final : public rclcpp::Node
{
public:
  Mi0802SenxorNode()
  : Node("mi0802_senxor_node")
  {
    device_path_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "mi0802_thermal_optical_frame");
    topic_name_ = declare_parameter<std::string>("topic_name", "/thermal/image_raw");
    reconnect_delay_sec_ = declare_parameter<double>("reconnect_delay_sec", 2.0);
    publish_rate_limit_ = declare_parameter<double>("publish_rate_limit", 0.0);
    flip_horizontal_ = declare_parameter<bool>("flip_horizontal", false);
    flip_vertical_ = declare_parameter<bool>("flip_vertical", false);
    rotate_180_ = declare_parameter<bool>("rotate_180", false);

    if (device_path_.empty() || frame_id_.empty() || topic_name_.empty()) {
      throw std::invalid_argument("device, frame_id, and topic_name must not be empty");
    }
    if (reconnect_delay_sec_ <= 0.0 || publish_rate_limit_ < 0.0) {
      throw std::invalid_argument(
              "reconnect_delay_sec must be positive and publish_rate_limit must be non-negative");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_name_, rclcpp::SensorDataQoS());
    device_ = std::make_unique<Mi0802Device>(device_path_);
    worker_ = std::thread(&Mi0802SenxorNode::run, this);
    RCLCPP_INFO(
      get_logger(), "MI0802 driver configured for %s; publishing 80x62 32FC1 Celsius on %s",
      device_path_.c_str(), topic_name_.c_str());
  }

  ~Mi0802SenxorNode() override
  {
    stop_requested_.store(true);
    reconnect_waiter_.notify();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void run()
  {
    std::size_t previous_invalid_count = 0;
    while (!stop_requested_.load() && rclcpp::ok()) {
      try {
        device_->open_device();
        RCLCPP_INFO(get_logger(), "Opened MI0802 serial device %s", device_path_.c_str());
        device_->initialize();
        device_->start_stream();
        previous_invalid_count = device_->invalid_message_count();
        last_publish_ = std::chrono::steady_clock::time_point{};
        RCLCPP_INFO(get_logger(), "MI0802 initialized with header mode enabled; streaming started");

        while (!stop_requested_.load() && rclcpp::ok()) {
          ThermalFrame frame;
          const bool received = device_->read_frame(frame, std::chrono::milliseconds(500));
          const auto invalid_count = device_->invalid_message_count();
          if (invalid_count > previous_invalid_count) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Discarded %zu corrupt or invalid MI0802 serial message(s); parser resynchronized",
              invalid_count - previous_invalid_count);
            previous_invalid_count = invalid_count;
          }
          if (!received) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "No complete MI0802 frame received in the last 500 ms");
            continue;
          }
          if (!should_publish()) {
            continue;
          }
          auto values = orient_frame(frame, flip_horizontal_, flip_vertical_, rotate_180_);
          publisher_->publish(make_image(values, now(), frame_id_));
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "MI0802 connection/stream error: %s", error.what());
      }

      device_->close_device();
      if (!stop_requested_.load() && rclcpp::ok()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000, "Reconnecting to MI0802 in %.2f seconds",
          reconnect_delay_sec_);
        reconnect_waiter_.wait_for(
          std::chrono::duration<double>(reconnect_delay_sec_), stop_requested_);
      }
    }
    device_->close_device();
  }

  bool should_publish()
  {
    const auto current = std::chrono::steady_clock::now();
    if (publish_rate_limit_ == 0.0) {
      last_publish_ = current;
      return true;
    }
    const auto minimum_period = std::chrono::duration<double>(1.0 / publish_rate_limit_);
    if (last_publish_.time_since_epoch().count() != 0 && current - last_publish_ < minimum_period) {
      return false;
    }
    last_publish_ = current;
    return true;
  }

  std::string device_path_;
  std::string frame_id_;
  std::string topic_name_;
  double reconnect_delay_sec_ = 2.0;
  double publish_rate_limit_ = 0.0;
  bool flip_horizontal_ = false;
  bool flip_vertical_ = false;
  bool rotate_180_ = false;
  std::atomic_bool stop_requested_{false};
  ReconnectWaiter reconnect_waiter_;
  std::unique_ptr<Mi0802Device> device_;
  std::thread worker_;
  std::chrono::steady_clock::time_point last_publish_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

}  // namespace mi0802_senxor_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mi0802_senxor_driver::Mi0802SenxorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mi0802_senxor_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
