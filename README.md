# ROS 2 initiator drone

This workspace is split into a top-level control package and sensor packages:

- `src/drone_control`: main drone launch/orchestration package.
- `src/sensors/mlx90640_node`: C++ ROS 2 driver for an MLX90640 32x24 thermal array over Linux I2C, plus an optional thermal-on-camera overlay.

`mlx90640_node` contains the Apache-2.0 Melexis calibration API and does not depend on Python, CircuitPython, or a virtual environment.

## Run on the Raspberry Pi

Verify the sensor is visible, normally at `0x33`:

```bash
sudo i2cdetect -y 1
```

Build the top-level control package and its workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

For Orbbec Gemini E / Dabai-style depth camera setup, read `DEPTH_CAMERA_DRIVER_SETUP.md`.

Launch the full drone graph:

```bash
ros2 launch drone_control drone_launch.py
```

Launch with rosbridge for the frontend:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true
```

The default launch starts the MLX90640 thermal node and publishes raw thermal frames at `/thermal/image_raw`. The frontend uses this thermal-only stream.

To also start the optional Orbbec camera and RGB thermal overlay, pass both camera flags and tune overlay transparency at launch:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true start_thermal_overlay:=true overlay_alpha:=0.45
```

For direct low-level sensor testing, you can still launch the sensor package by itself:

```bash
ros2 launch mlx90640_node mlx90640_launch.py
```

The thermal node publishes calibrated Celsius pixels as `sensor_msgs/Image` (`32FC1`, width 32, height 24) at `thermal/image_raw`, plus the sensor ambient temperature at `thermal/ambient`. Use `src/sensors/mlx90640_node/config/params.yaml` or ROS parameters to select another I2C device/address, sensor refresh rate, and emissivity:

```bash
ros2 run mlx90640_node mlx90640_node --ros-args --params-file src/sensors/mlx90640_node/config/params.yaml
```

The executing user must be permitted to open `/dev/i2c-1`, usually by membership in the `i2c` group. The `python3-smbus`, `i2c-tools`, CircuitPython, and `rpi-lgpio` installations are not required by this C++ node.
