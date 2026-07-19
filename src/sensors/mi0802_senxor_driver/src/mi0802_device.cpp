#include "mi0802_senxor_driver/mi0802_device.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <poll.h>
#include <sstream>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace mi0802_senxor_driver
{
namespace
{

constexpr std::array<std::uint8_t, 4> kPrefix{{' ', ' ', ' ', '#'}};
constexpr std::size_t kOuterHeaderSize = 8;
constexpr std::size_t kCommandSize = 4;
constexpr std::size_t kChecksumSize = 4;
constexpr std::size_t kMi08GfraDataSize = 10240;
constexpr std::size_t kMi08HeaderOffset = 160;
constexpr std::size_t kMi08PayloadOffset = 320;
constexpr std::chrono::milliseconds kOperationTimeout{3000};

std::optional<unsigned int> parse_hex(const std::uint8_t * bytes, std::size_t size)
{
  unsigned int value = 0;
  for (std::size_t index = 0; index < size; ++index) {
    const auto byte = bytes[index];
    unsigned int digit = 0;
    if (byte >= '0' && byte <= '9') {
      digit = byte - '0';
    } else if (byte >= 'A' && byte <= 'F') {
      digit = byte - 'A' + 10U;
    } else if (byte >= 'a' && byte <= 'f') {
      digit = byte - 'a' + 10U;
    } else {
      return std::nullopt;
    }
    value = (value << 4U) | digit;
  }
  return value;
}

std::vector<std::uint8_t> command_bytes(
  const char * command, std::uint8_t address, std::optional<std::uint8_t> value)
{
  std::ostringstream stream;
  stream << "   #" << (value ? "000C" : "000A") << command << std::uppercase <<
    std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned int>(address);
  if (value) {
    stream << std::setw(2) << static_cast<unsigned int>(*value);
  }
  stream << "XXXX";
  const auto text = stream.str();
  return {text.begin(), text.end()};
}

std::string serial_error(const std::string & device, int error)
{
  std::ostringstream message;
  if (error == ENOENT || error == ENODEV) {
    message << "MI0802 device not found at " << device;
  } else if (error == EACCES || error == EPERM) {
    message << "Permission denied opening " << device <<
      "; add the ROS user to the dialout group and log in again";
  } else if (error == EBUSY || error == EAGAIN) {
    message << "MI0802 device " << device << " is already in use";
  } else {
    message << "Could not open MI0802 device " << device << ": " << std::strerror(error);
  }
  return message.str();
}

std::uint16_t little_endian_u16(const std::uint8_t * bytes)
{
  return static_cast<std::uint16_t>(bytes[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U);
}

}  // namespace

void SerialMessageParser::append(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
  constexpr std::size_t kMaximumBufferedBytes = 131072;
  if (buffer_.size() > kMaximumBufferedBytes) {
    const auto excess = buffer_.size() - kMaximumBufferedBytes;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
    ++invalid_message_count_;
  }
}

void SerialMessageParser::append(const std::vector<std::uint8_t> & data)
{
  append(data.data(), data.size());
}

std::optional<SerialMessage> SerialMessageParser::next()
{
  while (true) {
    discard_to_prefix();
    if (buffer_.size() < kOuterHeaderSize) {
      return std::nullopt;
    }

    const auto body_size = parse_hex(buffer_.data() + 4, 4);
    if (!body_size || *body_size < kCommandSize + kChecksumSize) {
      reject_current_message();
      continue;
    }
    const std::size_t total_size = kOuterHeaderSize + *body_size;
    if (buffer_.size() < total_size) {
      return std::nullopt;
    }

    const auto checksum = parse_hex(buffer_.data() + total_size - kChecksumSize, kChecksumSize);
    bool command_valid = true;
    for (std::size_t index = kOuterHeaderSize; index < kOuterHeaderSize + kCommandSize; ++index) {
      const auto byte = buffer_[index];
      command_valid = command_valid &&
        ((byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z'));
    }
    std::uint32_t calculated = 0;
    for (std::size_t index = 4; index < total_size - kChecksumSize; ++index) {
      calculated += buffer_[index];
    }
    if (!checksum || !command_valid || (calculated & 0xFFFFU) != *checksum) {
      reject_current_message();
      continue;
    }

    SerialMessage message;
    message.command.assign(
      buffer_.begin() + static_cast<std::ptrdiff_t>(kOuterHeaderSize),
      buffer_.begin() + static_cast<std::ptrdiff_t>(kOuterHeaderSize + kCommandSize));
    const auto data_begin = kOuterHeaderSize + kCommandSize;
    const auto data_end = total_size - kChecksumSize;
    message.data.assign(
      buffer_.begin() + static_cast<std::ptrdiff_t>(data_begin),
      buffer_.begin() + static_cast<std::ptrdiff_t>(data_end));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total_size));
    return message;
  }
}

void SerialMessageParser::reset()
{
  buffer_.clear();
  invalid_message_count_ = 0;
}

std::size_t SerialMessageParser::invalid_message_count() const noexcept
{
  return invalid_message_count_;
}

void SerialMessageParser::discard_to_prefix()
{
  const auto found = std::search(buffer_.begin(), buffer_.end(), kPrefix.begin(), kPrefix.end());
  if (found == buffer_.end()) {
    const auto keep = std::min(buffer_.size(), kPrefix.size() - 1);
    if (buffer_.size() > keep) {
      buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
      ++invalid_message_count_;
    }
    return;
  }
  if (found != buffer_.begin()) {
    buffer_.erase(buffer_.begin(), found);
    ++invalid_message_count_;
  }
}

void SerialMessageParser::reject_current_message()
{
  const auto discard = std::min(buffer_.size(), kPrefix.size());
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(discard));
  ++invalid_message_count_;
}

Mi0802Device::Mi0802Device(std::string device)
: device_(std::move(device))
{
}

Mi0802Device::~Mi0802Device()
{
  close_device();
}

void Mi0802Device::open_device()
{
  close_device();
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw ProtocolError(serial_error(device_, errno));
  }
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw ProtocolError(serial_error(device_, error));
  }
  if (::ioctl(fd_, TIOCEXCL) != 0) {
    const int error = errno;
    ::close(fd_);
    fd_ = -1;
    throw ProtocolError(serial_error(device_, error));
  }

  termios settings{};
  if (::tcgetattr(fd_, &settings) != 0) {
    const int error = errno;
    close_device();
    throw ProtocolError("Could not read serial settings for " + device_ + ": " + std::strerror(error));
  }
  ::cfmakeraw(&settings);
  ::cfsetispeed(&settings, B115200);
  ::cfsetospeed(&settings, B115200);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
    const int error = errno;
    close_device();
    throw ProtocolError("Could not configure serial device " + device_ + ": " + std::strerror(error));
  }
  ::tcflush(fd_, TCIOFLUSH);
  parser_.reset();
  invalid_frame_count_ = 0;
  streaming_ = false;
}

void Mi0802Device::initialize()
{
  if (!is_open()) {
    throw ProtocolError("Cannot initialize MI0802: serial device is not open");
  }

  auto frame_mode = read_register(0xB1, kOperationTimeout);
  if ((frame_mode & 0x02U) != 0U) {
    write_register(0xB1, static_cast<std::uint8_t>(frame_mode & ~0x02U), kOperationTimeout);
  }

  const auto firmware_major_minor = read_register(0xB2, kOperationTimeout);
  const auto firmware_build = read_register(0xB3, kOperationTimeout);
  const auto sensor_type = read_register(0xBA, kOperationTimeout);
  (void)read_register(0xBB, kOperationTimeout);
  (void)read_register(0x33, kOperationTimeout);
  if (sensor_type != 4U && sensor_type != 5U && sensor_type != 8U) {
    std::ostringstream message;
    message << "Unsupported SenXor type " << static_cast<unsigned int>(sensor_type) <<
      " (expected MI0802 type 4, 5, or 8); firmware " <<
      static_cast<unsigned int>((firmware_major_minor >> 4U) & 0x0FU) << '.' <<
      static_cast<unsigned int>(firmware_major_minor & 0x0FU) << '.' <<
      static_cast<unsigned int>(firmware_build);
    throw ProtocolError(message.str());
  }

  auto frame_format = read_register(0x31, kOperationTimeout);
  if ((frame_format & 0x07U) != 0U) {
    frame_format = static_cast<std::uint8_t>(frame_format & ~0x07U);
    write_register(0x31, frame_format, kOperationTimeout);
  }

  frame_mode = read_register(0xB1, kOperationTimeout);
  const auto temperature_mode = static_cast<std::uint8_t>(frame_mode & ~(0x20U | 0x80U));
  if (temperature_mode != frame_mode) {
    write_register(0xB1, temperature_mode, kOperationTimeout);
  }
  streaming_ = false;
}

void Mi0802Device::start_stream()
{
  auto frame_mode = read_register(0xB1, kOperationTimeout);
  frame_mode = static_cast<std::uint8_t>((frame_mode | 0x02U) & ~(0x20U | 0x80U));
  write_register(0xB1, frame_mode, kOperationTimeout);
  streaming_ = true;
}

void Mi0802Device::stop_stream()
{
  if (!is_open()) {
    return;
  }
  auto frame_mode = read_register(0xB1, std::chrono::milliseconds(500));
  if ((frame_mode & 0x02U) != 0U) {
    write_register(
      0xB1, static_cast<std::uint8_t>(frame_mode & ~0x02U), std::chrono::milliseconds(500));
  }
  streaming_ = false;
}

void Mi0802Device::close_device() noexcept
{
  if (fd_ < 0) {
    return;
  }
  if (streaming_) {
    try {
      stop_stream();
    } catch (...) {
      // The descriptor is still closed below; disconnects commonly make the stop command fail.
    }
  }
  ::close(fd_);
  fd_ = -1;
  streaming_ = false;
  invalid_frame_count_ = 0;
  parser_.reset();
}

bool Mi0802Device::read_frame(ThermalFrame & frame, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    while (const auto message = parser_.next()) {
      if (decode_frame(*message, frame)) {
        return true;
      }
      if (message->command == "GFRA") {
        ++invalid_frame_count_;
      }
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      return false;
    }
    (void)read_serial(remaining);
  }
  return false;
}

bool Mi0802Device::is_open() const noexcept
{
  return fd_ >= 0;
}

std::size_t Mi0802Device::invalid_message_count() const noexcept
{
  return parser_.invalid_message_count() + invalid_frame_count_;
}

const std::string & Mi0802Device::device_path() const noexcept
{
  return device_;
}

bool Mi0802Device::decode_frame(const SerialMessage & message, ThermalFrame & frame)
{
  if (message.command != "GFRA" || message.data.size() != kMi08GfraDataSize) {
    return false;
  }
  for (std::size_t index = 0; index < frame.header.size(); ++index) {
    frame.header[index] = little_endian_u16(
      message.data.data() + kMi08HeaderOffset + index * sizeof(std::uint16_t));
  }
  for (std::size_t index = 0; index < frame.temperatures.size(); ++index) {
    const auto raw = little_endian_u16(
      message.data.data() + kMi08PayloadOffset + index * sizeof(std::uint16_t));
    const double celsius = static_cast<double>(raw) / 10.0 - 273.15;
    frame.temperatures[index] = static_cast<float>(std::round(celsius * 10.0) / 10.0);
  }
  return true;
}

std::uint8_t Mi0802Device::read_register(
  std::uint8_t address, std::chrono::milliseconds timeout)
{
  write_serial(command_bytes("RREG", address, std::nullopt), timeout);
  const auto response = wait_for_message("RREG", timeout);
  if (response.data.size() != 2) {
    throw ProtocolError("Unsupported RREG response length from MI0802");
  }
  const auto value = parse_hex(response.data.data(), response.data.size());
  if (!value || *value > 0xFFU) {
    throw ProtocolError("Invalid RREG response from MI0802");
  }
  return static_cast<std::uint8_t>(*value);
}

void Mi0802Device::write_register(
  std::uint8_t address, std::uint8_t value, std::chrono::milliseconds timeout)
{
  write_serial(command_bytes("WREG", address, value), timeout);
  const auto response = wait_for_message("WREG", timeout);
  if (!response.data.empty()) {
    throw ProtocolError("Unsupported WREG response from MI0802");
  }
}

SerialMessage Mi0802Device::wait_for_message(
  const std::string & command, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    while (const auto message = parser_.next()) {
      if (message->command == command) {
        return *message;
      }
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      break;
    }
    (void)read_serial(remaining);
  }
  throw ProtocolError("Initialization timeout waiting for MI0802 " + command + " response");
}

bool Mi0802Device::read_serial(std::chrono::milliseconds timeout)
{
  pollfd descriptor{fd_, POLLIN, 0};
  const auto bounded_timeout = std::max<long long>(timeout.count(), 0);
  const int result = ::poll(&descriptor, 1, static_cast<int>(bounded_timeout));
  if (result < 0) {
    if (errno == EINTR) {
      return false;
    }
    throw ProtocolError("MI0802 serial poll failed: " + std::string(std::strerror(errno)));
  }
  if (result == 0) {
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    throw ProtocolError("MI0802 sensor disconnected from " + device_);
  }
  std::array<std::uint8_t, 16384> bytes{};
  const auto count = ::read(fd_, bytes.data(), bytes.size());
  if (count > 0) {
    parser_.append(bytes.data(), static_cast<std::size_t>(count));
    return true;
  }
  if (count == 0) {
    throw ProtocolError("MI0802 sensor disconnected from " + device_);
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    return false;
  }
  throw ProtocolError("MI0802 serial read failed: " + std::string(std::strerror(errno)));
}

void Mi0802Device::write_serial(
  const std::vector<std::uint8_t> & bytes, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      throw ProtocolError("Timeout writing initialization command to MI0802");
    }
    pollfd descriptor{fd_, POLLOUT, 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw ProtocolError("MI0802 disconnected while writing initialization command");
    }
    const auto count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      throw ProtocolError("MI0802 serial write failed: " + std::string(std::strerror(errno)));
    }
  }
}

}  // namespace mi0802_senxor_driver
