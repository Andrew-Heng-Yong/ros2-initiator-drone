# ROS 2 initiator drone

This workspace is split into a top-level control package and sensor packages:

- `src/drone_control`: main drone launch/orchestration package.
- `src/sensors/mi0802_senxor_driver`: C++ ROS 2 driver for a Meridian Innovation MI0802 SenXor over USB CDC ACM.
- `src/sensors/mlx90640_node`: C++ ROS 2 driver for an MLX90640 32x24 thermal array over Linux I2C, plus an optional thermal-on-camera overlay.
- `src/sensors/mpu6050_node`: C++ ROS 2 driver for an MPU6050 accelerometer/gyroscope over Linux I2C.

`mlx90640_node` contains the Apache-2.0 Melexis calibration API and does not depend on Python, CircuitPython, or a virtual environment.

## Run on the Raspberry Pi

Verify the MI0802 serial device and the MPU6050 at `0x68` are visible:

```bash
ls -l /dev/ttyACM0 /dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM-if00
sudo i2cdetect -y 1
```

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

The frontend starts the Orbbec depth camera at 640x480 5 fps and performs the depth thermal overlay in the browser by combining `/camera/depth/image_raw` with `/thermal/image_raw`. The dashboard subscribes to `/camera/depth/camera_info` and uses it for the depth FOV when available, falling back to H67 x V53.6 degrees. When the Orbbec camera is enabled, the launch starts depth first and waits for the first depth topic before starting the MI0802 node, so thermal only appears as an overlay source after the depth stream is running.

To start the MPU6050 with the drone graph, pass `start_imu:=true`. The node defaults to `/dev/i2c-1`, address `0x68`, publishes raw IMU samples on `/imu/data_raw`, and publishes the chip temperature on `/imu/temperature`:

```bash
ros2 launch drone_control drone_launch.py start_imu:=true
```

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

For direct low-level MI0802 testing, launch the sensor package by itself:

```bash
ros2 launch mi0802_senxor_driver mi0802_senxor_launch.py
```

The thermal node publishes calibrated Celsius pixels as `sensor_msgs/Image` (`32FC1`, width 80, height 62) at `/thermal/image_raw`. Override the default `/dev/ttyACM0` path with the stable target path when desired:

```bash
ros2 launch mi0802_senxor_driver mi0802_senxor_launch.py device:=/dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM-if00
```

The ROS user needs serial access, normally through membership in `dialout`. See
`src/sensors/mi0802_senxor_driver/README.md` for parameters and verification commands. The
MLX90640 driver remains available and can be restored in `drone_launch.py` or launched
directly with `ros2 launch mlx90640_node mlx90640_launch.py`.

For direct MPU6050 testing:

```bash
ros2 launch mpu6050_node mpu6050_launch.py
```

The executing user must be permitted to open `/dev/ttyACM0` (normally via `dialout`) and `/dev/i2c-1` for the MPU6050 (normally via `i2c`). No Python SenXor runtime is required by the C++ thermal node.
