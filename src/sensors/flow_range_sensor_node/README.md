# PMW3901 optical flow + VL53L1X 4 m range driver

This ROS 2 Jazzy package drives the ALIENTEK/OpenEDV ATK-PMW3901 combined
module directly through Linux SPI and I2C. It has no Python runtime dependency.

## Raspberry Pi wiring

Use the module's `VCC3.0` pin, not `VUSB` or `+BATT`.

| Module signal | Raspberry Pi signal | Physical pin |
| --- | --- | --- |
| `GND` | Ground | 6 |
| `VCC3.0` | 3.3 V | 1 or 17 |
| `E_MOSI/PB15` | SPI0 MOSI / GPIO10 | 19 |
| `E_MISO/PB14` | SPI0 MISO / GPIO9 | 21 |
| `E_SCK/PB13` | SPI0 SCLK / GPIO11 | 23 |
| `E_CS3/PA8` | SPI0 CE1 / GPIO7 | 26 |
| `E_SDA/PB4` | I2C1 SDA / GPIO2 | 3 |
| `E_SCL/PB5` | I2C1 SCL / GPIO3 | 5 |

The defaults match `/dev/spidev0.1` and `/dev/i2c-1`. The module has already
been positively identified when SPI registers read `0x49`, `0x00`, `0xB6` and
I2C address `0x29` appears in `i2cdetect`.

## Published topics

- `/optical_flow/raw` (`flow_range_sensor_node/msg/OpticalFlow`): signed X/Y
  pixel-count deltas, motion flag, surface quality, shutter, integration time,
  and the latest non-stale downward range.
- `/range/down` (`sensor_msgs/msg/Range`): VL53L1X distance in metres. An
  invalid VL53L1X status is published as `NaN` and described in diagnostics.
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`): connectivity,
  read-error counts, flow illumination/quality, and VL53L1X signal status.

The node deliberately does not publish `/odom`, body velocity, or TF. Raw flow
counts require mounting orientation and scale calibration before an estimator
can safely turn them into velocity. `flow_rotation` supports 0, 90, 180, and
270 degrees so X/Y can be aligned with the airframe.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2-initiator-drone
colcon build --packages-select flow_range_sensor_node
source install/setup.bash
ros2 launch flow_range_sensor_node flow_range_sensor_launch.py
```

Verify output while moving the illuminated module over a textured surface:

```bash
ros2 topic hz /optical_flow/raw
ros2 topic echo /optical_flow/raw
ros2 topic echo /range/down
ros2 topic echo /diagnostics
```

The executing user needs access to SPI (commonly `dialout`) and I2C (commonly
`i2c`). For an immediate session fix, add the user to both groups and log out
and back in:

```bash
sudo usermod -aG dialout,i2c "$USER"
```

The bundled VL53L1X logic is derived from Pololu's BSD-3-Clause library, and
the PMW3901 initialization sequence is derived from Pimoroni's MIT-licensed
driver. Their licenses are installed under
`share/flow_range_sensor_node/third_party`.
