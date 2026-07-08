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

On MLX90640 modules with a `PS` protocol-select pin, connect `PS` to ground so the module uses I2C mode.

Build the top-level control package and its workspace dependencies:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

If colcon still looks for the old `src/mlx90640_node` path after the package
move to `src/sensors/mlx90640_node`, clear the stale CMake package build caches
and rebuild:

```bash
cd ~/ros2-initiator-drone
rm -rf build/mlx90640_node build/drone_control install/mlx90640_node install/drone_control
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

For Orbbec Gemini E / Dabai-style depth camera setup, read `DEPTH_CAMERA_DRIVER_SETUP.md`.

Launch the top-level drone graph:

```bash
ros2 launch drone_control drone_launch.py
```

Launch with rosbridge for the frontend:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true
```

The frontend starts the Orbbec depth camera at 640x480 5 fps and performs the depth thermal overlay in the browser by combining `/camera/depth/image_raw` with `/thermal/image_raw`. The dashboard subscribes to `/camera/depth/camera_info` and uses it for the depth FOV when available, falling back to H67 x V53.6 degrees. When the Orbbec camera is enabled, the launch starts depth first and waits for the first depth topic before starting the MLX90640 node, so thermal only appears as an overlay source after the depth stream is running.

To start the Orbbec camera alongside the thermal node for browser-side overlay,
pass the camera flag:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_depth_camera:=true
```

The optional ROS-side overlay executable is still available for experiments, but
it is not required by the dashboard. Build it only if you need the ROS topic
`/camera/thermal_overlay/image_raw`:

```bash
sudo apt install -y ros-jazzy-cv-bridge libopencv-dev
colcon build --packages-up-to drone_control --cmake-args -DBUILD_THERMAL_OVERLAY=ON
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
