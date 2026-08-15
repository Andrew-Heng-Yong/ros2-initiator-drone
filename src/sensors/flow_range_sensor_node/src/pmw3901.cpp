#include "flow_range_sensor_node/pmw3901.hpp"

// The PMW3901 initialization table and orientation mapping are derived from
// Pimoroni's MIT-licensed pmw3901-python driver. See third_party/pmw3901/LICENSE.txt.

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <initializer_list>
#include <linux/spi/spidev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <utility>
#include <unistd.h>

namespace flow_range_sensor_node
{
namespace
{

using namespace std::chrono_literals;

std::runtime_error system_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

int16_t signed_word(uint8_t low, uint8_t high)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
}

}  // namespace

Pmw3901::Pmw3901(const std::string & device, uint32_t speed_hz, uint8_t mode)
: speed_hz_(speed_hz), mode_(mode)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw system_error("open(" + device + ")");
  }

  uint8_t bits = 8;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
  {
    const auto error = system_error("configure SPI device");
    ::close(fd_);
    fd_ = -1;
    throw error;
  }
}

Pmw3901::~Pmw3901()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

uint8_t Pmw3901::read_register(uint8_t address)
{
  uint8_t command = address & 0x7F;
  uint8_t dummy = 0;
  uint8_t value = 0;
  std::array<spi_ioc_transfer, 2> transfers{};
  transfers[0].tx_buf = reinterpret_cast<uint64_t>(&command);
  transfers[0].len = 1;
  transfers[0].speed_hz = speed_hz_;
  transfers[0].bits_per_word = 8;
  transfers[0].delay_usecs = 5;
  transfers[1].tx_buf = reinterpret_cast<uint64_t>(&dummy);
  transfers[1].rx_buf = reinterpret_cast<uint64_t>(&value);
  transfers[1].len = 1;
  transfers[1].speed_hz = speed_hz_;
  transfers[1].bits_per_word = 8;

  if (::ioctl(fd_, SPI_IOC_MESSAGE(2), transfers.data()) < 0) {
    throw system_error("PMW3901 register read");
  }
  std::this_thread::sleep_for(1us);
  return value;
}

void Pmw3901::write_register(uint8_t address, uint8_t value)
{
  std::array<uint8_t, 2> data{static_cast<uint8_t>(address | 0x80), value};
  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<uint64_t>(data.data());
  transfer.len = data.size();
  transfer.speed_hz = speed_hz_;
  transfer.bits_per_word = 8;

  if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    throw system_error("PMW3901 register write");
  }
  std::this_thread::sleep_for(20us);
}

void Pmw3901::initialize(int rotation_degrees)
{
  std::this_thread::sleep_for(50ms);

  write_register(0x3A, 0x5A);
  std::this_thread::sleep_for(20ms);
  for (uint8_t address = 0x02; address <= 0x06; ++address) {
    static_cast<void>(read_register(address));
  }

  apply_initialization_sequence();

  product_id_ = read_register(0x00);
  revision_id_ = read_register(0x01);
  inverse_product_id_ = read_register(0x5F);
  if (product_id_ != 0x49 || (revision_id_ != 0x00 && revision_id_ != 0x01) ||
    inverse_product_id_ != 0xB6)
  {
    throw std::runtime_error("unexpected PMW3901 product, revision, or inverse product ID");
  }
  set_rotation(rotation_degrees);
}

void Pmw3901::apply_initialization_sequence()
{
  const auto write_many = [this](
    std::initializer_list<std::pair<uint8_t, uint8_t>> values)
    {
      for (const auto & [address, value] : values) {
        write_register(address, value);
      }
    };

  write_many({
      {0x7F, 0x00}, {0x55, 0x01}, {0x50, 0x07}, {0x7F, 0x0E}, {0x43, 0x10}});
  write_register(0x48, (read_register(0x67) & 0x80) ? 0x04 : 0x02);
  write_many({{0x7F, 0x00}, {0x51, 0x7B}, {0x50, 0x00}, {0x55, 0x00}, {0x7F, 0x0E}});

  if (read_register(0x73) == 0x00) {
    int c1 = read_register(0x70);
    int c2 = read_register(0x71);
    if (c1 <= 28) {
      c1 += 14;
    }
    if (c1 > 28) {
      c1 += 11;
    }
    c1 = std::clamp(c1, 0, 0x3F);
    c2 = (c2 * 45) / 100;
    write_many({{0x7F, 0x00}, {0x61, 0xAD}, {0x51, 0x70}, {0x7F, 0x0E}});
    write_register(0x70, static_cast<uint8_t>(c1));
    write_register(0x71, static_cast<uint8_t>(c2));
  }

  write_many({
      {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05},
      {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34},
      {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
      {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
      {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78},
      {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
      {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F},
      {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
      {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
      {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
      {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
      {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}});
  std::this_thread::sleep_for(10ms);
  write_many({
      {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0},
      {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00},
      {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80}});
  std::this_thread::sleep_for(240ms);
  write_many({{0x7F, 0x14}, {0x6F, 0x1C}, {0x7F, 0x00}});
}

void Pmw3901::set_rotation(int rotation_degrees)
{
  uint8_t orientation = 0;
  switch (rotation_degrees) {
    case 0:
      orientation = 0xE0;
      break;
    case 90:
      orientation = 0x40;
      break;
    case 180:
      orientation = 0x80;
      break;
    case 270:
      orientation = 0x20;
      break;
    default:
      throw std::invalid_argument("flow_rotation must be one of 0, 90, 180, or 270 degrees");
  }
  write_register(0x7F, 0x00);
  write_register(0x5B, orientation);
}

FlowSample Pmw3901::read_motion_burst()
{
  uint8_t command = 0x16;
  uint8_t dummy[12]{};
  uint8_t data[12]{};
  std::array<spi_ioc_transfer, 2> transfers{};
  transfers[0].tx_buf = reinterpret_cast<uint64_t>(&command);
  transfers[0].len = 1;
  transfers[0].speed_hz = speed_hz_;
  transfers[0].bits_per_word = 8;
  transfers[0].delay_usecs = 35;
  transfers[1].tx_buf = reinterpret_cast<uint64_t>(dummy);
  transfers[1].rx_buf = reinterpret_cast<uint64_t>(data);
  transfers[1].len = sizeof(data);
  transfers[1].speed_hz = speed_hz_;
  transfers[1].bits_per_word = 8;

  if (::ioctl(fd_, SPI_IOC_MESSAGE(2), transfers.data()) < 0) {
    throw system_error("PMW3901 motion burst");
  }

  FlowSample sample;
  sample.motion_detected = (data[0] & 0x80) != 0;
  sample.observation = data[1];
  sample.delta_x = signed_word(data[2], data[3]);
  sample.delta_y = signed_word(data[4], data[5]);
  sample.quality = data[6];
  sample.raw_data_sum = data[7];
  sample.raw_data_max = data[8];
  sample.raw_data_min = data[9];
  sample.shutter = static_cast<uint16_t>(
    (static_cast<uint16_t>(data[10]) << 8) | data[11]);
  return sample;
}

}  // namespace flow_range_sensor_node
