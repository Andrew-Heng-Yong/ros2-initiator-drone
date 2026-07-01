# Human Pose Detection

RGB-only ROS 2 node for MoveNet Lightning INT8 TensorFlow Lite inference.

No thermal camera, MLX90640 data, depth fusion, or multi-sensor confidence logic is used.

## Run

```bash
ros2 launch human_pose_detection movenet_pose_launch.py \
  model_path:=/path/to/movenet_lightning_int8.tflite \
  image_topic:=/camera/image_raw
```

The top-level drone launch uses `/camera/color/image_raw` by default:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true \
  pose_model_path:=/path/to/movenet_lightning_int8.tflite
```

## Topics

- Subscribes: `/camera/image_raw` by default, configurable with `image_topic`
- Publishes keypoints: `/human_pose/keypoints`
- Publishes detection flag: `/human_pose/person_detected`
- Publishes annotated image: `/human_pose/debug_image`

## Keypoint Payload

`/human_pose/keypoints` is a `std_msgs/msg/Float32MultiArray`:

```text
[
  detected,
  bbox_x,
  bbox_y,
  bbox_w,
  bbox_h,
  nose_x, nose_y, nose_score,
  left_eye_x, left_eye_y, left_eye_score,
  ...
]
```

Coordinates are in source image pixels. A missing keypoint has `x=-1`, `y=-1`, and its raw score.

## Runtime Dependencies

Install either `tflite_runtime` or TensorFlow with Lite support, plus OpenCV and `cv_bridge`.
