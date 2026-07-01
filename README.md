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

## Launch

Run the RGB camera, MoveNet pose node, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  pose_model_path:=/path/to/movenet_lightning_int8.tflite
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
