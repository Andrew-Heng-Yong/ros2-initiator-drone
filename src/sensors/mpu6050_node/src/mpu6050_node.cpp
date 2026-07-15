#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"

namespace {
constexpr uint8_t kRegisterPowerManagement1 = 0x6B;
constexpr uint8_t kRegisterSampleRateDivider = 0x19;
constexpr uint8_t kRegisterConfig = 0x1A;
constexpr uint8_t kRegisterGyroConfig = 0x1B;
constexpr uint8_t kRegisterAccelConfig = 0x1C;
constexpr uint8_t kRegisterAccelOut = 0x3B;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = kPi / 180.0;

int rangeBits(int value, const std::array<int, 4> & allowed, const std::string & name) {
  const auto found = std::find(allowed.begin(), allowed.end(), value);
  if (found == allowed.end()) {
    throw std::runtime_error(name + " has an unsupported MPU6050 range");
  }
  return static_cast<int>(std::distance(allowed.begin(), found));
}

double accelScale(int range_g) {
  switch (range_g) {
    case 2: return 16384.0;
    case 4: return 8192.0;
    case 8: return 4096.0;
    case 16: return 2048.0;
    default: return 16384.0;
  }
}

double gyroScale(int range_dps) {
  switch (range_dps) {
    case 250: return 131.0;
    case 500: return 65.5;
    case 1000: return 32.8;
    case 2000: return 16.4;
    default: return 131.0;
  }
}

int16_t readBigEndianInt16(const uint8_t * data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}
}  // namespace

class Mpu6050Node : public rclcpp::Node {
public:
  Mpu6050Node() : Node("mpu6050_node") {
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    i2c_address_ = declare_parameter<int>("i2c_address", 0x68);
    frame_id_ = declare_parameter<std::string>("frame_id", "mpu6050_link");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    const auto temperature_topic = declare_parameter<std::string>("temperature_topic", "/imu/temperature");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    gyro_range_dps_ = declare_parameter<int>("gyro_range_dps", 250);
    accel_range_g_ = declare_parameter<int>("accel_range_g", 2);

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be greater than zero");
    }

    accel_scale_ = accelScale(accel_range_g_);
    gyro_scale_ = gyroScale(gyro_range_dps_);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      temperature_topic, rclcpp::SensorDataQoS());

    openDevice();
    configureSensor();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishSample(); });

    RCLCPP_INFO(
      get_logger(),
      "MPU6050 on %s address 0x%02x publishing %s at %.1f Hz",
      i2c_device_.c_str(),
      i2c_address_,
      imu_topic.c_str(),
      publish_rate_hz_);
  }

  ~Mpu6050Node() override {
    if (fd_ >= 0) close(fd_);
  }

private:
  void openDevice() {
    fd_ = open(i2c_device_.c_str(), O_RDWR | O_CLOEXEC);
    if (fd_ < 0) {
      throw std::runtime_error("cannot open I2C device " + i2c_device_);
    }
    if (ioctl(fd_, I2C_SLAVE, i2c_address_) < 0) {
      throw std::runtime_error("cannot select MPU6050 I2C address");
    }
  }

  void configureSensor() {
    writeRegister(kRegisterPowerManagement1, 0x00);
    writeRegister(kRegisterSampleRateDivider, sampleRateDivider());
    writeRegister(kRegisterConfig, 0x03);
    writeRegister(
      kRegisterGyroConfig,
      static_cast<uint8_t>(rangeBits(gyro_range_dps_, {250, 500, 1000, 2000}, "gyro_range_dps") << 3));
    writeRegister(
      kRegisterAccelConfig,
      static_cast<uint8_t>(rangeBits(accel_range_g_, {2, 4, 8, 16}, "accel_range_g") << 3));
  }

  uint8_t sampleRateDivider() const {
    const auto divider = std::clamp<int>(static_cast<int>(std::round(1000.0 / publish_rate_hz_)) - 1, 0, 255);
    return static_cast<uint8_t>(divider);
  }

  void writeRegister(uint8_t reg, uint8_t value) {
    const uint8_t bytes[2] = {reg, value};
    if (write(fd_, bytes, sizeof(bytes)) != static_cast<ssize_t>(sizeof(bytes))) {
      throw std::runtime_error("failed to write MPU6050 register");
    }
  }

  void readRegisters(uint8_t start_register, uint8_t * buffer, std::size_t length) {
    if (write(fd_, &start_register, 1) != 1) {
      throw std::runtime_error("failed to select MPU6050 register");
    }
    if (read(fd_, buffer, length) != static_cast<ssize_t>(length)) {
      throw std::runtime_error("failed to read MPU6050 registers");
    }
  }

  void publishSample() {
    uint8_t raw[14] = {};
    try {
      readRegisters(kRegisterAccelOut, raw, sizeof(raw));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
      return;
    }

    const auto now = get_clock()->now();
    const double ax = static_cast<double>(readBigEndianInt16(&raw[0])) / accel_scale_ * kGravity;
    const double ay = static_cast<double>(readBigEndianInt16(&raw[2])) / accel_scale_ * kGravity;
    const double az = static_cast<double>(readBigEndianInt16(&raw[4])) / accel_scale_ * kGravity;
    const double temperature_c = static_cast<double>(readBigEndianInt16(&raw[6])) / 340.0 + 36.53;
    const double gx = static_cast<double>(readBigEndianInt16(&raw[8])) / gyro_scale_ * kDegToRad;
    const double gy = static_cast<double>(readBigEndianInt16(&raw[10])) / gyro_scale_ * kDegToRad;
    const double gz = static_cast<double>(readBigEndianInt16(&raw[12])) / gyro_scale_ * kDegToRad;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now;
    imu.header.frame_id = frame_id_;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = gx;
    imu.angular_velocity.y = gy;
    imu.angular_velocity.z = gz;
    imu.linear_acceleration.x = ax;
    imu.linear_acceleration.y = ay;
    imu.linear_acceleration.z = az;
    imu_pub_->publish(imu);

    sensor_msgs::msg::Temperature temperature;
    temperature.header.stamp = now;
    temperature.header.frame_id = frame_id_;
    temperature.temperature = temperature_c;
    temperature.variance = 0.0;
    temperature_pub_->publish(temperature);
  }

  int fd_{-1};
  int i2c_address_{0x68};
  int gyro_range_dps_{250};
  int accel_range_g_{2};
  double publish_rate_hz_{100.0};
  double accel_scale_{16384.0};
  double gyro_scale_{131.0};
  std::string i2c_device_{"/dev/i2c-1"};
  std::string frame_id_{"mpu6050_link"};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Mpu6050Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mpu6050_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
