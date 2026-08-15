#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class TwoWire
{
public:
  TwoWire() = default;
  ~TwoWire();

  TwoWire(const TwoWire &) = delete;
  TwoWire & operator=(const TwoWire &) = delete;

  bool begin(const std::string & device);
  void close();
  bool isOpen() const {return fd_ >= 0;}
  const std::string & lastError() const {return last_error_;}
  uint64_t errorCount() const {return error_count_;}

  void beginTransmission(uint8_t address);
  size_t write(uint8_t value);
  uint8_t endTransmission();
  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  int read();

private:
  bool selectAddress(uint8_t address);

  int fd_{-1};
  uint8_t address_{0};
  std::vector<uint8_t> tx_buffer_;
  std::vector<uint8_t> rx_buffer_;
  size_t rx_offset_{0};
  std::string last_error_;
  uint64_t error_count_{0};
};

extern TwoWire Wire;
