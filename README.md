# ROS 2 initiator drone

This workspace is split into a top-level launch package and perception packages:

- `src/drone_control`: main drone launch/orchestration package.
- `src/perception/human_pose_detection`: RGB-only MoveNet Lightning INT8 human pose detection.

## Build

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

The pose node needs `cv_bridge`, OpenCV, and either `tflite_runtime` or TensorFlow Lite support available in the Python environment.
The thermal camera launch path needs `v4l2_camera`:

```bash
sudo apt install ros-jazzy-v4l2-camera
```

## Launch

Run the RGB camera, MoveNet pose node, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  pose_model_path:=/path/to/movenet_lightning_int8.tflite
```

Run only the 256x192 YUYV thermal camera and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=false \
  start_pose:=false \
  start_thermal_camera:=true
```

## Tune Without Rebuild

Launch defaults are loaded from:

```bash
install/drone_control/share/drone_control/config/drone_settings.yaml
```

Edit that installed YAML and restart the launch to tune camera FPS, pose
thresholds, and inference FPS without rebuilding:

```yaml
camera:
  color_width: 640
  color_height: 480
  color_fps: 5

pose:
  confidence_threshold: 0.2
  min_confident_keypoints: 5
  max_inference_fps: 5.0

thermal_camera:
  device: /dev/video0
  width: 256
  height: 192
  fps: 25
  pixel_format: YUYV
  output_encoding: yuv422_yuy2
```

You can also keep a separate file and point the launch at it:

```bash
DRONE_SETTINGS_FILE=/home/andrew/drone_settings.yaml \
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_camera:=true
```

Run only the pose node against an existing camera topic:

```bash
ros2 launch human_pose_detection movenet_pose_launch.py \
  model_path:=/path/to/movenet_lightning_int8.tflite \
  image_topic:=/camera/image_raw
```

## Pose Topics

- Input image: `/camera/image_raw` by default, `/camera/color/image_raw` in the top-level drone launch
- Keypoints: `/human_pose/keypoints`
- Detection flag: `/human_pose/person_detected`
- Debug image: `/human_pose/debug_image`

## Thermal Topics

- Image: `/thermal/image_raw`
- Camera info: `/thermal/camera_info`
