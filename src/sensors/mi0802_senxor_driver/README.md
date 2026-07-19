# MI0802 SenXor ROS 2 driver

This package supports the Meridian Innovation MI0802 SenXor over its native USB CDC ACM
connection. The default device is `/dev/ttyACM0`; the stable path observed on the target is
`/dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM-if00` and can be supplied with the `device`
launch argument.

The user running ROS must be able to open the serial device. On common Linux distributions,
add that user to `dialout`, log out, and log back in:

```bash
sudo usermod -aG dialout "$USER"
```

Build and launch:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mi0802_senxor_driver
source install/setup.bash
ros2 launch mi0802_senxor_driver mi0802_senxor_launch.py
```

To use the stable path:

```bash
ros2 launch mi0802_senxor_driver mi0802_senxor_launch.py \
  device:=/dev/serial/by-id/usb-Nuvoton_USB_Virtual_COM-if00
```

The node publishes `/thermal/image_raw` as `sensor_msgs/msg/Image`, encoding `32FC1`, width
80, height 62. Each float is a calibrated temperature in degrees Celsius. Parameters are
`device`, `frame_id`, `topic_name`, `reconnect_delay_sec`, `publish_rate_limit`,
`flip_horizontal`, `flip_vertical`, and `rotate_180`. A publish rate limit of `0.0` publishes
every received frame.

Verify the stream:

```bash
ros2 node list
ros2 topic list
ros2 topic info /thermal/image_raw
ros2 topic hz /thermal/image_raw
ros2 topic echo /thermal/image_raw --once
```

The top-level `drone_control` launch uses this driver while preserving the existing thermal
topic contract. To switch back to MLX90640, replace the MI0802 process in
`src/drone_control/launch/drone_launch.py` with the previous `mlx90640_node` command and
rebuild. The retained `mlx90640_node` dependency supplies the shared cropper and overlay;
the MLX90640 driver itself remains installed and can still be launched directly.

The serial framing, register commands, header layout, and deci-Kelvin conversion are a
minimal C++ port of Meridian Innovation's Apache-2.0 `pysenxor-lite` implementation. The
sensor firmware performs its factory-calibrated temperature conversion; this driver sets
deci-Kelvin output, converts with `raw / 10 - 273.15`, and rounds to 0.1 degrees Celsius as
the reference library does.
