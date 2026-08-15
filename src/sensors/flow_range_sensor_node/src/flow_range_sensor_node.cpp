#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "VL53L1X.h"
#include "Wire.h"
#include "flow_range_sensor_node/msg/optical_flow.hpp"
#include "flow_range_sensor_node/pmw3901.hpp"

namespace flow_range_sensor_node
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template<typename RepT>
std::chrono::nanoseconds rate_period(RepT rate_hz)
{
  if (rate_hz <= 0.0) {
    throw std::invalid_argument("publish rates must be greater than zero");
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

bool valid_range_status(VL53L1X::RangeStatus status)
{
  return status == VL53L1X::RangeValid ||
         status == VL53L1X::RangeValidMinRangeClipped ||
         status == VL53L1X::RangeValidNoWrapCheckFail;
}

}  // namespace

class FlowRangeSensorNode : public rclcpp::Node
{
public:
  FlowRangeSensorNode()
  : Node("flow_range_sensor_node")
  {
    enable_flow_ = declare_parameter<bool>("enable_flow", true);
    enable_range_ = declare_parameter<bool>("enable_range", true);
    const auto spi_device = declare_parameter<std::string>("spi_device", "/dev/spidev0.1");
    const auto spi_speed_hz = declare_parameter<int>("spi_speed_hz", 400000);
    const auto spi_mode = declare_parameter<int>("spi_mode", 0);
    const auto flow_rotation = declare_parameter<int>("flow_rotation", 0);
    flow_rate_hz_ = declare_parameter<double>("flow_rate_hz", 100.0);
    flow_frame_id_ = declare_parameter<std::string>("flow_frame_id", "flow_link");
    const auto flow_topic = declare_parameter<std::string>("flow_topic", "/optical_flow/raw");
    minimum_flow_quality_ = declare_parameter<int>("minimum_flow_quality", 25);
    dark_shutter_threshold_ = declare_parameter<int>("dark_shutter_threshold", 8000);

    const auto i2c_device = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    const auto i2c_address = declare_parameter<int>("i2c_address", 0x29);
    range_rate_hz_ = declare_parameter<double>("range_rate_hz", 20.0);
    const auto timing_budget_us = declare_parameter<int>("range_timing_budget_us", 50000);
    range_frame_id_ = declare_parameter<std::string>("range_frame_id", "range_link");
    const auto range_topic = declare_parameter<std::string>("range_topic", "/range/down");
    range_min_m_ = declare_parameter<double>("range_min_m", 0.04);
    range_max_m_ = declare_parameter<double>("range_max_m", 4.0);
    range_fov_rad_ = declare_parameter<double>("range_field_of_view", 0.471);
    range_stale_after_s_ = declare_parameter<double>("range_stale_after", 0.20);
    const auto diagnostics_topic =
      declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    if (!enable_flow_ && !enable_range_) {
      throw std::invalid_argument("at least one of enable_flow or enable_range must be true");
    }
    if ((enable_flow_ && flow_rate_hz_ <= 0.0) || (enable_range_ && range_rate_hz_ <= 0.0)) {
      throw std::invalid_argument("enabled sensor rates must be greater than zero");
    }
    if (spi_speed_hz <= 0 || spi_mode < 0 || spi_mode > 3) {
      throw std::invalid_argument("invalid SPI speed or mode");
    }
    if (flow_rotation != 0 && flow_rotation != 90 &&
      flow_rotation != 180 && flow_rotation != 270)
    {
      throw std::invalid_argument("flow_rotation must be one of 0, 90, 180, or 270");
    }
    if (minimum_flow_quality_ < 0 || minimum_flow_quality_ > 255 ||
      dark_shutter_threshold_ < 0 || dark_shutter_threshold_ > 8191)
    {
      throw std::invalid_argument("invalid flow quality or shutter threshold");
    }
    if (i2c_address != 0x29) {
      throw std::invalid_argument(
              "this VL53L1X driver expects the power-on I2C address 0x29 (41)");
    }
    if (timing_budget_us < 20000 || timing_budget_us > 1000000) {
      throw std::invalid_argument("range_timing_budget_us must be between 20000 and 1000000");
    }
    if (range_min_m_ < 0.0 || range_max_m_ <= range_min_m_ || range_fov_rad_ <= 0.0 ||
      range_stale_after_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid range limits or stale timeout");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);

    if (enable_flow_) {
      flow_publisher_ = create_publisher<msg::OpticalFlow>(flow_topic, sensor_qos);
      flow_sensor_ = std::make_unique<Pmw3901>(
        spi_device, static_cast<uint32_t>(spi_speed_hz), static_cast<uint8_t>(spi_mode));
      flow_sensor_->initialize(flow_rotation);
      RCLCPP_INFO(
        get_logger(), "PMW3901 connected on %s (ID 0x%02X, revision 0x%02X)",
        spi_device.c_str(), flow_sensor_->product_id(), flow_sensor_->revision_id());
      flow_timer_ = create_wall_timer(
        rate_period(flow_rate_hz_), [this]() {publish_flow();});
    }

    if (enable_range_) {
      range_publisher_ = create_publisher<sensor_msgs::msg::Range>(range_topic, sensor_qos);
      if (!i2c_bus_.begin(i2c_device)) {
        throw std::runtime_error(i2c_bus_.lastError());
      }
      range_sensor_.setBus(&i2c_bus_);
      range_sensor_.setTimeout(250);
      if (!range_sensor_.init(true)) {
        throw std::runtime_error(
                "VL53L1X initialization failed on " + i2c_device + " at 0x29: " +
                i2c_bus_.lastError());
      }
      if (!range_sensor_.setDistanceMode(VL53L1X::Long)) {
        throw std::runtime_error("VL53L1X rejected long-distance mode");
      }
      if (!range_sensor_.setMeasurementTimingBudget(static_cast<uint32_t>(timing_budget_us))) {
        throw std::runtime_error("VL53L1X rejected range_timing_budget_us");
      }
      const auto intermeasurement_ms = static_cast<uint32_t>(
        std::max(1.0, std::ceil(1000.0 / range_rate_hz_)));
      if (intermeasurement_ms * 1000U < static_cast<uint32_t>(timing_budget_us)) {
        throw std::invalid_argument(
                "range_rate_hz period must not be shorter than range_timing_budget_us");
      }
      range_sensor_.startContinuous(intermeasurement_ms);
      range_started_ = true;
      RCLCPP_INFO(
        get_logger(), "VL53L1X connected on %s at 0x29 (long mode, %d us budget)",
        i2c_device.c_str(), timing_budget_us);
      const auto poll_period = std::min(
        rate_period(range_rate_hz_),
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(10)));
      range_timer_ = create_wall_timer(poll_period, [this]() {publish_range_if_ready();});
    }

    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});
  }

  ~FlowRangeSensorNode() override
  {
    if (range_started_) {
      range_sensor_.stopContinuous();
    }
  }

private:
  void publish_flow()
  {
    try {
      const auto stamp = now();
      const auto sample = flow_sensor_->read_motion_burst();
      msg::OpticalFlow message;
      message.header.stamp = stamp;
      message.header.frame_id = flow_frame_id_;
      message.motion_detected = sample.motion_detected;
      message.delta_x = sample.delta_x;
      message.delta_y = sample.delta_y;
      message.quality = sample.quality;
      message.observation = sample.observation;
      message.raw_data_sum = sample.raw_data_sum;
      message.raw_data_max = sample.raw_data_max;
      message.raw_data_min = sample.raw_data_min;
      message.shutter = sample.shutter;
      message.integration_time = last_flow_stamp_.has_value() ?
        static_cast<float>((stamp - *last_flow_stamp_).seconds()) :
        static_cast<float>(1.0 / flow_rate_hz_);
      last_flow_stamp_ = stamp;

      message.range_valid = latest_range_stamp_.has_value() && latest_range_valid_ &&
        (stamp - *latest_range_stamp_).seconds() <= range_stale_after_s_;
      message.ground_distance = message.range_valid ? latest_range_m_ :
        std::numeric_limits<float>::quiet_NaN();
      flow_publisher_->publish(message);

      last_flow_sample_ = sample;
      flow_consecutive_errors_ = 0;
      ++flow_sample_count_;
      if (sample.motion_detected) {
        ++flow_motion_count_;
      }
    } catch (const std::exception & error) {
      ++flow_error_count_;
      ++flow_consecutive_errors_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "PMW3901 read failed: %s", error.what());
    }
  }

  void publish_range_if_ready()
  {
    const uint64_t errors_before_poll = i2c_bus_.errorCount();
    if (!range_sensor_.dataReady()) {
      if (range_sensor_.last_status != 0 || i2c_bus_.errorCount() != errors_before_poll) {
        ++range_error_count_;
        ++range_consecutive_errors_;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "VL53L1X poll failed: %s",
          i2c_bus_.lastError().c_str());
      }
      return;
    }

    const auto stamp = now();
    const uint64_t errors_before_read = i2c_bus_.errorCount();
    const uint16_t millimeters = range_sensor_.read(false);
    if (range_sensor_.last_status != 0 || i2c_bus_.errorCount() != errors_before_read) {
      ++range_error_count_;
      ++range_consecutive_errors_;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "VL53L1X read failed: %s",
        i2c_bus_.lastError().c_str());
      return;
    }
    range_consecutive_errors_ = 0;

    const auto status = range_sensor_.ranging_data.range_status;
    const float meters = static_cast<float>(millimeters) / 1000.0F;
    const bool valid = valid_range_status(status) &&
      meters >= static_cast<float>(range_min_m_) && meters <= static_cast<float>(range_max_m_);

    sensor_msgs::msg::Range message;
    message.header.stamp = stamp;
    message.header.frame_id = range_frame_id_;
    message.radiation_type = sensor_msgs::msg::Range::INFRARED;
    message.field_of_view = static_cast<float>(range_fov_rad_);
    message.min_range = static_cast<float>(range_min_m_);
    message.max_range = static_cast<float>(range_max_m_);
    message.range = valid ? meters : std::numeric_limits<float>::quiet_NaN();
    range_publisher_->publish(message);

    latest_range_stamp_ = stamp;
    latest_range_m_ = meters;
    latest_range_valid_ = valid;
    latest_range_status_ = status;
    latest_signal_mcps_ = range_sensor_.ranging_data.peak_signal_count_rate_MCPS;
    latest_ambient_mcps_ = range_sensor_.ranging_data.ambient_count_rate_MCPS;
    ++range_sample_count_;
    if (!valid) {
      ++range_invalid_count_;
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    if (enable_flow_) {
      DiagnosticStatus status;
      status.name = get_fully_qualified_name() + std::string(": PMW3901");
      status.hardware_id = "PMW3901-0x49";
      if (flow_consecutive_errors_ > 0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "SPI read errors detected";
      } else if (flow_sample_count_ > 0 &&
        last_flow_sample_.shutter >= dark_shutter_threshold_)
      {
        status.level = DiagnosticStatus::WARN;
        status.message = "surface illumination is too low";
      } else if (flow_sample_count_ > 0 && last_flow_sample_.motion_detected &&
        last_flow_sample_.quality < minimum_flow_quality_)
      {
        status.level = DiagnosticStatus::WARN;
        status.message = "motion detected with low surface quality";
      } else {
        status.level = DiagnosticStatus::OK;
        status.message = "flow sensor operational";
      }
      status.values.push_back(key_value("samples", std::to_string(flow_sample_count_)));
      status.values.push_back(key_value("motion samples", std::to_string(flow_motion_count_)));
      status.values.push_back(key_value("errors", std::to_string(flow_error_count_)));
      status.values.push_back(key_value("quality", std::to_string(last_flow_sample_.quality)));
      status.values.push_back(key_value("shutter", std::to_string(last_flow_sample_.shutter)));
      array.status.push_back(std::move(status));
    }

    if (enable_range_) {
      DiagnosticStatus status;
      status.name = get_fully_qualified_name() + std::string(": VL53L1X");
      status.hardware_id = "VL53L1X-0x29";
      if (range_consecutive_errors_ > 0) {
        status.level = DiagnosticStatus::ERROR;
        status.message = "I2C errors detected";
      } else if (range_sample_count_ == 0) {
        status.level = DiagnosticStatus::WARN;
        status.message = "waiting for first range sample";
      } else if (!latest_range_valid_) {
        status.level = DiagnosticStatus::WARN;
        status.message = VL53L1X::rangeStatusToString(latest_range_status_);
      } else {
        status.level = DiagnosticStatus::OK;
        status.message = "range sensor operational";
      }
      status.values.push_back(key_value("samples", std::to_string(range_sample_count_)));
      status.values.push_back(key_value("invalid samples", std::to_string(range_invalid_count_)));
      status.values.push_back(key_value("errors", std::to_string(range_error_count_)));
      status.values.push_back(key_value("distance m", std::to_string(latest_range_m_)));
      status.values.push_back(key_value("signal MCPS", std::to_string(latest_signal_mcps_)));
      status.values.push_back(key_value("ambient MCPS", std::to_string(latest_ambient_mcps_)));
      array.status.push_back(std::move(status));
    }

    diagnostics_publisher_->publish(array);
  }

  bool enable_flow_{true};
  bool enable_range_{true};
  bool range_started_{false};
  double flow_rate_hz_{100.0};
  double range_rate_hz_{20.0};
  double range_min_m_{0.04};
  double range_max_m_{4.0};
  double range_fov_rad_{0.471};
  double range_stale_after_s_{0.20};
  int minimum_flow_quality_{25};
  int dark_shutter_threshold_{8000};
  std::string flow_frame_id_;
  std::string range_frame_id_;

  std::unique_ptr<Pmw3901> flow_sensor_;
  TwoWire i2c_bus_;
  VL53L1X range_sensor_;

  rclcpp::Publisher<msg::OpticalFlow>::SharedPtr flow_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr flow_timer_;
  rclcpp::TimerBase::SharedPtr range_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::optional<rclcpp::Time> last_flow_stamp_;
  std::optional<rclcpp::Time> latest_range_stamp_;
  FlowSample last_flow_sample_;
  float latest_range_m_{std::numeric_limits<float>::quiet_NaN()};
  bool latest_range_valid_{false};
  VL53L1X::RangeStatus latest_range_status_{VL53L1X::None};
  float latest_signal_mcps_{0.0F};
  float latest_ambient_mcps_{0.0F};
  uint64_t flow_sample_count_{0};
  uint64_t flow_motion_count_{0};
  uint64_t flow_error_count_{0};
  uint64_t flow_consecutive_errors_{0};
  uint64_t range_sample_count_{0};
  uint64_t range_invalid_count_{0};
  uint64_t range_error_count_{0};
  uint64_t range_consecutive_errors_{0};
};

}  // namespace flow_range_sensor_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<flow_range_sensor_node::FlowRangeSensorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("flow_range_sensor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
