# Depth Camera Driver Setup

This project expects an Orbbec Gemini E / Dabai-style camera through the ROS 2
`orbbec_camera` driver. Use the older `main` branch of the Orbbec ROS 2 driver
for these legacy/OpenNI cameras. The newer `v2-main` branch can detect the USB
device but may fail to open this camera family.

## Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-rosdep \
  git \
  build-essential \
  cmake \
  libopencv-dev \
  ros-jazzy-camera-info-manager \
  ros-jazzy-cv-bridge \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-image-transport \
  ros-jazzy-image-publisher
```

Use `humble` instead of `jazzy` in package names if the robot is running ROS 2
Humble.

## Build The Older Orbbec Driver

```bash
sudo rosdep init 2>/dev/null || true
rosdep update

mkdir -p ~/orbbec_ws/src
cd ~/orbbec_ws/src
git clone -b main https://github.com/orbbec/OrbbecSDK_ROS2.git orbbec_camera

cd ~/orbbec_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

Do not use `-b v2-main` for the Gemini E / Dabai-style camera unless you have
confirmed that branch opens your exact camera.

## Install USB Permission Rules

Install the Orbbec udev rules so the driver can open the USB device without
`sudo`:

```bash
cd ~/orbbec_ws/src/orbbec_camera/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the camera. If it is connected through a hub, plug it
directly into the Raspberry Pi while testing.

## Verify The Camera

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node
```

Expected output should mention a Gemini E / Orbbec device. You can also check:

```bash
lsusb | grep -i "2bc5\|orbbec"
```

## Manual Camera Launch Test

The drone launch file starts color at 640x480 10 fps and leaves depth disabled.
Test the same settings manually:

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini_e.launch.py \
  color_width:=640 \
  color_height:=480 \
  color_fps:=10 \
  enable_depth:=false \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=10 \
  enable_ir:=false
```

Verify topics:

```bash
ros2 topic list | grep -E "camera|image|camera_info"
ros2 topic hz /camera/color/image_raw
```

## Dashboard Human Tracking Debug Stream

The dashboard renders `/human_pose/debug_image`, which is produced by the
RGB-only TensorFlow Lite human box tracker from `/camera/color/image_raw`. Source both the
Orbbec workspace and the drone workspace, then start the camera explicitly.

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
source ~/ros2-initiator-drone/install/setup.bash
ros2 launch drone_control drone_launch.py \
  start_rosbridge:=true \
  start_camera:=true
```

The human tracking stack uses:

```text
input:  /camera/color/image_raw
output: /human_pose/keypoints  # box payload for compatibility with older frontend wiring
output: /human_pose/person_detected
output: /human_pose/debug_image
```

## Troubleshooting

If the build stops with a missing CMake package such as
`camera_info_managerConfig.cmake` or `image_publisherConfig.cmake`, first let
`rosdep` resolve and install all package dependencies from the source manifests:

```bash
cd ~/orbbec_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-select orbbec_camera
source install/setup.bash
```

Ask `rosdep` which apt package provides a missing dependency:

```bash
rosdep resolve image_publisher --rosdistro jazzy
rosdep resolve camera_info_manager --rosdistro jazzy
```

Search the local apt package index by package name:

```bash
apt-cache search ros-jazzy-image-publisher
apt-cache search ros-jazzy-camera-info-manager
```

For a literal `*Config.cmake` file search, install `apt-file` once:

```bash
sudo apt install -y apt-file
sudo apt-file update
apt-file search image_publisherConfig.cmake
apt-file search camera_info_managerConfig.cmake
```

To manually install known missing packages and rebuild only the driver:

```bash
sudo apt install -y ros-jazzy-camera-info-manager ros-jazzy-image-publisher
cd ~/orbbec_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select orbbec_camera
source install/setup.bash
```

If launch fails with:

```text
Failed to get depth profile: Invalid input, No matched video stream profile found!
Stream: OB_STREAM_DEPTH, Width: 1280, Height: 720, FPS: 10
```

check the `Available profiles` printed by the driver and launch with one of
those exact depth modes. On the Raspberry Pi test unit, the Gemini E connected
as USB2.0 and advertised `640x480 10fps Y11`, so the project default uses:

```bash
depth_width:=640 depth_height:=480 depth_fps:=10
```
