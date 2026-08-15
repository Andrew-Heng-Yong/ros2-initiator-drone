#pragma once

#include <chrono>
#include <cstdint>
#include <thread>

inline void delay(unsigned long milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

inline void delayMicroseconds(unsigned int microseconds)
{
  std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

inline uint32_t millis()
{
  static const auto start = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count());
}
