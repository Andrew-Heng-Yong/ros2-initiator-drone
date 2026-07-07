# ROS 2 initiator drone

This workspace is split into a top-level launch package and perception packages:

- `src/drone_control`: main drone launch/orchestration package.
- `src/perception/human_pose_detection`: RGB-only TensorFlow Lite human box tracking.

## Build

```bash
cd ~/ros2-initiator-drone
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to drone_control
source install/setup.bash
```

The human tracker needs `cv_bridge`, OpenCV, and either `tflite_runtime` or TensorFlow Lite support available in the Python environment.
The RGB launch uses the Orbbec driver workspace configured by `ORBBEC_SETUP`.

## Launch

Run the RGB camera, human box tracker, and rosbridge:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true
```

## Tune Without Rebuild

Launch defaults are loaded from:

```bash
install/drone_control/share/drone_control/config/drone_settings.yaml
```

The human box tracker also loads its own ROS parameter file:

```bash
install/human_pose_detection/share/human_pose_detection/config/human_box_tracker.yaml
```

Edit those installed YAML files and restart the launch to tune camera FPS,
crop pixels, human tracking thresholds, and inference FPS without rebuilding:

```yaml
camera:
  color_width: 640
  color_height: 360
  color_fps: 5
```

```yaml
human_box_tracker_node:
  ros__parameters:
    model_name: efficientdet_lite0_person_boxes
    model_path: /home/andrew/models/efficientdet_lite0.tflite
    image_topic: /camera/color/image_raw
    confidence_threshold: 0.35
    max_detections: 8
    crop_left_px: 29
    crop_right_px: 29
    crop_top_px: 0
    crop_bottom_px: 0
    track_iou_threshold: 0.3
    max_track_missed_frames: 5
    max_inference_fps: 2.0
    publish_debug_image: true
```

You can also keep a separate file and point the launch at it:

```bash
DRONE_SETTINGS_FILE=/home/andrew/drone_settings.yaml \
ros2 launch drone_control drone_launch.py start_rosbridge:=true start_camera:=true
```

Pass `human_tracking_params_file:=/home/andrew/human_box_tracker.yaml` if you
want a separate tracker parameter file outside the install tree.

Run only the human box tracker against an existing camera topic:

```bash
ros2 launch human_pose_detection movenet_pose_launch.py \
  params_file:=/path/to/human_box_tracker.yaml
```

## Human Tracking Topics

- Input image: `/camera/image_raw` by default, `/camera/color/image_raw` in the top-level drone launch
- Boxes: `/human_pose/keypoints`
- Detection flag: `/human_pose/person_detected`
- Debug image: `/human_pose/debug_image`
