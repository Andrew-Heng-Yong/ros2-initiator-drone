#include "Wire.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

TwoWire Wire;

TwoWire::~TwoWire()
{
  close();
}

bool TwoWire::begin(const std::string & device)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    last_error_ = "open(" + device + "): " + std::strerror(errno);
    ++error_count_;
    return false;
  }
  last_error_.clear();
  error_count_ = 0;
  return true;
}

void TwoWire::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool TwoWire::selectAddress(uint8_t address)
{
  if (fd_ < 0) {
    last_error_ = "I2C device is not open";
    ++error_count_;
    return false;
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    last_error_ = "I2C_SLAVE(0x";
    constexpr char hex[] = "0123456789ABCDEF";
    last_error_ += hex[(address >> 4) & 0x0F];
    last_error_ += hex[address & 0x0F];
    last_error_ += "): ";
    last_error_ += std::strerror(errno);
    ++error_count_;
    return false;
  }
  return true;
}

void TwoWire::beginTransmission(uint8_t address)
{
  address_ = address;
  tx_buffer_.clear();
}

size_t TwoWire::write(uint8_t value)
{
  tx_buffer_.push_back(value);
  return 1;
}

uint8_t TwoWire::endTransmission()
{
  if (!selectAddress(address_)) {
    return 4;
  }
  const ssize_t written = ::write(fd_, tx_buffer_.data(), tx_buffer_.size());
  if (written != static_cast<ssize_t>(tx_buffer_.size())) {
    last_error_ = "I2C write: ";
    last_error_ += written < 0 ? std::strerror(errno) : "short write";
    ++error_count_;
    return 4;
  }
  return 0;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  rx_buffer_.assign(quantity, 0);
  rx_offset_ = 0;
  if (!selectAddress(address)) {
    rx_buffer_.clear();
    return 0;
  }
  const ssize_t received = ::read(fd_, rx_buffer_.data(), rx_buffer_.size());
  if (received != static_cast<ssize_t>(rx_buffer_.size())) {
    last_error_ = "I2C read: ";
    last_error_ += received < 0 ? std::strerror(errno) : "short read";
    ++error_count_;
    rx_buffer_.clear();
    return 0;
  }
  return quantity;
}

int TwoWire::read()
{
  if (rx_offset_ >= rx_buffer_.size()) {
    return -1;
  }
  return rx_buffer_[rx_offset_++];
}
