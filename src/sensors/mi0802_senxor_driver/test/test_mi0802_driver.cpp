#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "mi0802_senxor_driver/frame_utils.hpp"
#include "mi0802_senxor_driver/mi0802_device.hpp"

namespace mi0802_senxor_driver
{
namespace
{

std::vector<std::uint8_t> make_gfra(std::uint16_t raw_temperature)
{
  std::vector<std::uint8_t> data(10240, 0);
  for (std::size_t index = 0; index < kPixelCount; ++index) {
    const auto offset = 320 + index * 2;
    data[offset] = static_cast<std::uint8_t>(raw_temperature & 0xFFU);
    data[offset + 1] = static_cast<std::uint8_t>(raw_temperature >> 8U);
  }
  const std::string length = "2808";
  const std::string command = "GFRA";
  unsigned int checksum = 0;
  for (const auto byte : length) {
    checksum += static_cast<unsigned char>(byte);
  }
  for (const auto byte : command) {
    checksum += static_cast<unsigned char>(byte);
  }
  for (const auto byte : data) {
    checksum += byte;
  }
  char checksum_text[5]{};
  std::snprintf(checksum_text, sizeof(checksum_text), "%04X", checksum & 0xFFFFU);
  std::string prefix = "   #" + length + command;
  std::vector<std::uint8_t> packet(prefix.begin(), prefix.end());
  packet.insert(packet.end(), data.begin(), data.end());
  packet.insert(packet.end(), checksum_text, checksum_text + 4);
  return packet;
}

TEST(SerialParser, ReassemblesPartialReadsAndDecodes80By62Frame)
{
  auto packet = make_gfra(2987);  // 25.55 C, rounded like pysenxor-lite to 25.6 C.
  SerialMessageParser parser;
  for (std::size_t offset = 0; offset < packet.size(); offset += 37) {
    const auto count = std::min<std::size_t>(37, packet.size() - offset);
    parser.append(packet.data() + offset, count);
  }
  const auto message = parser.next();
  ASSERT_TRUE(message.has_value());
  ThermalFrame frame;
  ASSERT_TRUE(Mi0802Device::decode_frame(*message, frame));
  EXPECT_EQ(frame.temperatures.size(), kFrameHeight * kFrameWidth);
  EXPECT_FLOAT_EQ(frame.temperatures.front(), 25.6F);
  EXPECT_FLOAT_EQ(frame.temperatures.back(), 25.6F);
}

TEST(SerialParser, InvalidHeaderOrChecksumDoesNotProduceFrame)
{
  auto bad_prefix = make_gfra(2987);
  bad_prefix[3] = '!';
  auto bad_checksum = make_gfra(2987);
  bad_checksum.back() = bad_checksum.back() == '0' ? '1' : '0';
  SerialMessageParser parser;
  parser.append(bad_prefix);
  EXPECT_FALSE(parser.next().has_value());
  parser.append(bad_checksum);
  EXPECT_FALSE(parser.next().has_value());
  EXPECT_GT(parser.invalid_message_count(), 0U);
}

TEST(ImageContract, Publishes32FC1WithExpectedDimensionsAndDataLength)
{
  ThermalFrame frame;
  auto values = orient_frame(frame, false, false, false);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  const auto image = make_image(values, stamp, "mi0802_thermal_optical_frame");
  EXPECT_EQ(image.height, 62U);
  EXPECT_EQ(image.width, 80U);
  EXPECT_EQ(image.encoding, "32FC1");
  EXPECT_EQ(image.step, 80U * sizeof(float));
  EXPECT_EQ(image.data.size(), 62U * 80U * sizeof(float));
  EXPECT_EQ(image.header.stamp.sec, 123);
}

TEST(Orientation, HorizontalVerticalAndRotate180AreApplied)
{
  ThermalFrame frame;
  for (std::size_t index = 0; index < frame.temperatures.size(); ++index) {
    frame.temperatures[index] = static_cast<float>(index);
  }
  const auto horizontal = orient_frame(frame, true, false, false);
  EXPECT_FLOAT_EQ(horizontal[0], 79.0F);
  EXPECT_FLOAT_EQ(horizontal[79], 0.0F);
  const auto vertical = orient_frame(frame, false, true, false);
  EXPECT_FLOAT_EQ(vertical[0], static_cast<float>(61 * 80));
  const auto rotated = orient_frame(frame, false, false, true);
  EXPECT_FLOAT_EQ(rotated[0], static_cast<float>(kPixelCount - 1));
  const auto cancelled = orient_frame(frame, true, true, true);
  EXPECT_FLOAT_EQ(cancelled[0], 0.0F);
}

TEST(ReconnectWaiter, DelayDoesNotBusyLoopAndShutdownInterruptsWait)
{
  ReconnectWaiter waiter;
  std::atomic_bool stop{false};
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(waiter.wait_for(std::chrono::milliseconds(60), stop));
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(40));

  auto future = std::async(std::launch::async, [&]() {
    return waiter.wait_for(std::chrono::seconds(5), stop);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop.store(true);
  waiter.notify();
  EXPECT_EQ(future.wait_for(std::chrono::milliseconds(250)), std::future_status::ready);
  EXPECT_TRUE(future.get());
}

}  // namespace
}  // namespace mi0802_senxor_driver
