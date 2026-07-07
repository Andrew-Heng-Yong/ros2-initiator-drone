# ROS 2 initiator drone

This workspace is centered on the top-level launch package:

- `src/drone_control`: main drone launch/orchestration package.

## Build

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

The thermal camera launch path needs `v4l2_camera`:

```bash
sudo apt install ros-jazzy-v4l2-camera
```

## Launch

Run the thermal camera, Orbbec depth stream, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  start_thermal_camera:=true
```

Run only the 256x192 YUYV thermal camera and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=false \
  start_thermal_camera:=true
```

## Tune Without Rebuild

Launch defaults are loaded from:

```bash
install/drone_control/share/drone_control/config/drone_settings.yaml
```

Edit that installed YAML and restart the launch to tune camera FPS without rebuilding:

```yaml
camera:
  color_width: 640
  color_height: 480
  color_fps: 5
  enable_depth: true
  depth_width: 640
  depth_height: 480
  depth_fps: 5

thermal_camera:
  device: /dev/video0
  width: 256
  height: 392
  fps: 25
  pixel_format: YUYV
  output_encoding: yuv422_yuy2
```

Some HikCamera-style thermal UVC devices expose the real 256x192 sensor image
inside a native 256x392 YUYV transport frame. The dashboard crops that transport
frame and displays the 256x192 thermal image.

You can also keep a separate file and point the launch at it:

```bash
DRONE_SETTINGS_FILE=/home/andrew/drone_settings.yaml \
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_camera:=true
```

## Thermal Topics

- Image: `/thermal/image_raw`
- Camera info: `/thermal/camera_info`
