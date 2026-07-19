#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mi0802_senxor_driver
{

constexpr std::size_t kFrameWidth = 80;
constexpr std::size_t kFrameHeight = 62;
constexpr std::size_t kPixelCount = kFrameWidth * kFrameHeight;
constexpr std::size_t kHeaderWordCount = 80;

struct SerialMessage
{
  std::string command;
  std::vector<std::uint8_t> data;
};

struct ThermalFrame
{
  std::array<std::uint16_t, kHeaderWordCount> header{};
  std::array<float, kPixelCount> temperatures{};
};

class ProtocolError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class SerialMessageParser
{
public:
  void append(const std::uint8_t * data, std::size_t size);
  void append(const std::vector<std::uint8_t> & data);
  std::optional<SerialMessage> next();
  void reset();
  std::size_t invalid_message_count() const noexcept;

private:
  void discard_to_prefix();
  void reject_current_message();

  std::vector<std::uint8_t> buffer_;
  std::size_t invalid_message_count_ = 0;
};

class Mi0802Device
{
public:
  explicit Mi0802Device(std::string device);
  ~Mi0802Device();

  Mi0802Device(const Mi0802Device &) = delete;
  Mi0802Device & operator=(const Mi0802Device &) = delete;

  void open_device();
  void initialize();
  void start_stream();
  void stop_stream();
  void close_device() noexcept;

  bool read_frame(ThermalFrame & frame, std::chrono::milliseconds timeout);
  bool is_open() const noexcept;
  std::size_t invalid_message_count() const noexcept;
  const std::string & device_path() const noexcept;

  static bool decode_frame(const SerialMessage & message, ThermalFrame & frame);

private:
  std::uint8_t read_register(std::uint8_t address, std::chrono::milliseconds timeout);
  void write_register(
    std::uint8_t address, std::uint8_t value, std::chrono::milliseconds timeout);
  SerialMessage wait_for_message(const std::string & command, std::chrono::milliseconds timeout);
  bool read_serial(std::chrono::milliseconds timeout);
  void write_serial(const std::vector<std::uint8_t> & bytes, std::chrono::milliseconds timeout);

  std::string device_;
  int fd_ = -1;
  bool streaming_ = false;
  std::size_t invalid_frame_count_ = 0;
  SerialMessageParser parser_;
};

}  // namespace mi0802_senxor_driver
