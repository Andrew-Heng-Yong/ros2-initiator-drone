#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace flow_range_sensor_node
{

struct FlowSample
{
  bool motion_detected{false};
  uint8_t observation{0};
  int16_t delta_x{0};
  int16_t delta_y{0};
  uint8_t quality{0};
  uint8_t raw_data_sum{0};
  uint8_t raw_data_max{0};
  uint8_t raw_data_min{0};
  uint16_t shutter{0};
};

class Pmw3901
{
public:
  Pmw3901(const std::string & device, uint32_t speed_hz, uint8_t mode = 0);
  ~Pmw3901();

  Pmw3901(const Pmw3901 &) = delete;
  Pmw3901 & operator=(const Pmw3901 &) = delete;

  void initialize(int rotation_degrees);
  FlowSample read_motion_burst();

  uint8_t product_id() const {return product_id_;}
  uint8_t revision_id() const {return revision_id_;}
  uint8_t inverse_product_id() const {return inverse_product_id_;}

private:
  uint8_t read_register(uint8_t address);
  void write_register(uint8_t address, uint8_t value);
  void apply_initialization_sequence();
  void set_rotation(int rotation_degrees);

  int fd_{-1};
  uint32_t speed_hz_{0};
  uint8_t mode_{0};
  uint8_t product_id_{0};
  uint8_t revision_id_{0};
  uint8_t inverse_product_id_{0};
};

}  // namespace flow_range_sensor_node
