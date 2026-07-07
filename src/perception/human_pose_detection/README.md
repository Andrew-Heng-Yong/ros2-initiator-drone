# Human Tracking

RGB-only ROS 2 nodes for TensorFlow Lite human tracking. The active launch path
uses a lightweight person-box detector so multiple humans can be marked in the
debug stream.

## Run

```bash
ros2 launch human_pose_detection human_box_tracker_launch.py \
  params_file:=/path/to/human_box_tracker.yaml
```

The default model selection is in the installed ROS parameter file:

- `share/human_pose_detection/config/human_box_tracker.yaml`
- `model_name: efficientdet_lite0_int8_person_boxes`
- `model_path: /home/andrew/ros2-initiator-drone/models/efficientdet_lite0_int8.tflite`

The top-level drone launch uses `/camera/color/image_raw` by default, captures
RGB at 640x360, and runs detection on a 582x360 center crop after removing 29 px
from each side:

```bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true
```

## Topics

- Subscribes: `/camera/color/image_raw` by default, configurable in `human_box_tracker.yaml`
- Publishes boxes: `/human_pose/keypoints`
- Publishes detection flag: `/human_pose/person_detected`
- Publishes annotated image: `/human_pose/debug_image`

## Box Payload

`/human_pose/keypoints` is a `std_msgs/msg/Float32MultiArray`:

```text
[
  person_count,
  person_1_track_id, person_1_x, person_1_y, person_1_w, person_1_h, person_1_score,
  person_2_track_id, person_2_x, person_2_y, person_2_w, person_2_h, person_2_score,
  ...
]
```

Coordinates are in source image pixels.

## Runtime Dependencies

Install OpenCV/cv_bridge and one TensorFlow Lite interpreter:

```bash
sudo apt install -y ros-jazzy-cv-bridge python3-opencv python3-numpy
```

Then install one of these, depending on what is available for the Pi OS/Python version:

```bash
python3 -m pip install --break-system-packages tflite-runtime
# or
python3 -m pip install --break-system-packages ai-edge-litert
# or, heavier:
python3 -m pip install --break-system-packages tensorflow
```

Quick check:

```bash
python3 -c "from tflite_runtime.interpreter import Interpreter; print('tflite ok')"
```

If you installed LiteRT instead, check:

```bash
python3 -c "from ai_edge_litert.interpreter import Interpreter; print('litert ok')"
```
