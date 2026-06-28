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

The drone launch file starts the camera at 1280x720 and 10 fps. Test the same
settings manually:

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini_e.launch.py \
  color_width:=1280 \
  color_height:=720 \
  color_fps:=10 \
  enable_depth:=true \
  depth_width:=1280 \
  depth_height:=720 \
  depth_fps:=5 \
  enable_ir:=false
```

Verify topics:

```bash
ros2 topic list | grep -E "camera|depth|image|camera_info"
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

## Dashboard Launch Integration

The frontend service sources:

```text
~/orbbec_ws/install/setup.bash
```

before launching the drone stack. If your Orbbec workspace is somewhere else,
set `ORBBEC_SETUP` before starting the dashboard:

```bash
export ORBBEC_SETUP=/path/to/orbbec_ws/install/setup.bash
cd ~/frontend-initiator-drone
npm start
```

The dashboard starts:

```bash
ros2 launch drone_control drone_launch.py start_rosbridge:=true overlay_alpha:=0.45
```

The ROS overlay node publishes:

```text
/camera/thermal_overlay/image_raw
```

The thermal overlay assumes the depth/color camera and MLX90640 are mounted at
the same position. It maps the MLX90640 field of view, 55 degrees horizontal by
35 degrees vertical, into the camera field of view, 67 degrees horizontal by
53.6 degrees vertical.

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
