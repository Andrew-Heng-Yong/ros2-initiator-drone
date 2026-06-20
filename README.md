# ROS 2 MLX90640 node

`src/mlx90640_node` is a C++ ROS 2 driver for an MLX90640 (32 × 24) thermal array over Linux I²C. It contains the Apache-2.0 Melexis calibration API and does not depend on Python, CircuitPython, or a virtual environment.

On the Raspberry Pi, verify the sensor is visible (normally at `0x33`):

```bash
sudo i2cdetect -y 1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select mlx90640_node
source install/setup.bash
ros2 launch mlx90640_node mlx90640_launch.py
```

It publishes calibrated Celsius pixels as `sensor_msgs/Image` (`32FC1`, width 32, height 24) at `thermal/image_raw`, plus the sensor ambient temperature at `thermal/ambient`. Use `config/params.yaml` or ROS parameters to select another I²C device/address, sensor refresh rate, and emissivity:

```bash
ros2 run mlx90640_node mlx90640_node --ros-args --params-file src/mlx90640_node/config/params.yaml
```

The executing user must be permitted to open `/dev/i2c-1` (usually by membership in the `i2c` group). The `python3-smbus`, `i2c-tools`, CircuitPython, and `rpi-lgpio` installations are not required by this C++ node.
