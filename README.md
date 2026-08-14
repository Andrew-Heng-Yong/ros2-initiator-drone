# ROS 2 initiator drone

This workspace is split into top-level control, localization, and sensor packages:

- `src/drone_control`: main drone launch/orchestration package.
- `src/sensors/mi0802_senxor_driver`: C++ ROS 2 driver for a Meridian Innovation MI0802 SenXor over USB CDC ACM.
- `src/sensors/mlx90640_node`: C++ ROS 2 driver for an MLX90640 32x24 thermal array over Linux I2C, plus an optional thermal-on-camera overlay.
- `src/sensors/mpu6050_node`: C++ ROS 2 driver for an MPU6050 accelerometer/gyroscope over Linux I2C.
- `src/localization/odom_node`: IMU dead-reckoning odometry publishing `/odom` and `odom -> base_link`.

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

The frontend performs the depth thermal overlay in the browser by combining `/camera/depth/image_raw` with `/thermal/image_raw`. The dashboard subscribes to `/camera/depth/camera_info` and uses it for the depth FOV when available, falling back to H67 x V53.6 degrees. The top-level graph can start the camera, IMU, odometry, cropper, and rosbridge concurrently. The MI0802 process waits only for the first depth topic before starting.

To start the MPU6050 with the drone graph, pass `start_imu:=true`. The node defaults to `/dev/i2c-1`, address `0x68`, publishes raw IMU samples on `/imu/data_raw`, and publishes the chip temperature on `/imu/temperature`:

```bash
ros2 launch drone_control drone_launch.py start_imu:=true
```

To start gyro odometry, pass `start_odom:=true`; this also starts the MPU6050 by default. No
camera stream is required. Keep the drone stationary while startup calibration collects 1000
gyro samples and estimates angular-rate bias. Stationarity is determined primarily from sample
variation, allowing a stable zero-rate sensor offset to be learned:

```bash
ros2 launch drone_control drone_launch.py \
  start_odom:=true
ros2 topic echo /odom
```

The node consumes only `/imu/data_raw`. Its stationary calibration normalizes the measured
acceleration magnitude to standard gravity, allowing integration to work with MPU-compatible
boards whose effective acceleration scale differs from register readback. It publishes completion
on `/odom/calibrated` once per second (with transient-local durability for native ROS subscribers)
and applies a ten-read rolling mean to gyro and acceleration before publishing bias-corrected gyro
values, integrated relative orientation, and diagnostic acceleration on
`/imu/data_calibrated`. By default it also removes the calibrated gravity reference and integrates
acceleration into three-dimensional `/odom` velocity and position. It does not infer stationarity
or apply zero-velocity resets from IMU data. Acceleration is lightly filtered before integration.
This gives existing clients a motion response but is IMU-only dead reckoning and position will
drift quickly. Parameters are configured in
`src/localization/odom_node/config/params.yaml`; replace the default IMU mount rotation with the
measured value before flight. See the package README for estimator limitations.

For a stationary bench setup, `odom_static_override:=true` bypasses calibration and publishes a
fixed, calibrated origin pose with observed position covariance. This makes visualization clients
such as the iPhone app report a complete tracked pose. Disable the override before the robot moves:

```bash
ros2 launch drone_control drone_launch.py \
  start_odom:=true odom_static_override:=true
```

`odom_quality_override:=true` is the less invasive visualization override: gyro orientation keeps
updating, but the node reports its unmeasured zero translation with low covariance so the iPhone
shows **Tracking**. It does not improve the underlying estimate.

The thermal cropper publishes the tight selected depth ROI rather than a full-size image padded
with zeros. Its output uses latest-only ROS QoS and defaults to `depth_output_decimation:=2`, which
keeps every second pixel in each axis and reduces the rosbridge depth payload by a further 4x.
CameraInfo dimensions and intrinsics are adjusted to match, so point-cloud geometry remains valid.
The depth camera remains on the project's verified 5 fps profile. Use
`depth_output_decimation:=1` when full cropped resolution is required.

Crop detection thresholds the native thermal pixels, groups occupied analysis cells into
8-connected components, selects the largest component (breaking ties by its true hot-pixel count),
and inflates only that component's real highlighted pixels. A three-frame hold prevents brief
thermal threshold dropouts from switching the depth stream between cropped and full-frame output.
Depth projection uses one contiguous cached depth-to-thermal lookup rather than a vector per
thermal pixel, keeping per-frame work bounded and cache-friendly.

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
