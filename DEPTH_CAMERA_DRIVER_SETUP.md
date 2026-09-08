# Gemini E camera profile

The installed legacy Orbbec ROS 2 main driver works with this Gemini E
(wrapper 1.5.15, SDK 1.10.35). Keep it in ~/orbbec_ws; verify legacy-device
compatibility before changing driver branches.

## Verified live profile

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 run orbbec_camera orbbec_camera_node --ros-args -r __node:=camera -r __ns:=/camera -p camera_name:=camera -p enable_color:=true -p color_width:=640 -p color_height:=360 -p color_fps:=15 -p color_format:=MJPG -p enable_depth:=true -p depth_width:=640 -p depth_height:=360 -p depth_fps:=15 -p depth_format:=Y11 -p enable_ir:=false -p enable_accel:=false -p enable_gyro:=false -p depth_registration:=true -p align_mode:=HW -p enable_frame_sync:=false -p enable_point_cloud:=false
```

The rebuild uses this profile. RGB and depth must both be 640×360: the camera's
factory calibration table has an exact pair at that resolution. The 640×480 pair
produced images but zero CameraInfo focal lengths. Do not substitute approximate
field-of-view intrinsics to hide this failure.

Verified factory RGB parameters for the installed camera:

| Parameter | Value |
|---|---:|
| fx, fy | 358.893646 px |
| cx | 320.212830 px |
| cy | 175.093048 px |

The application reads CameraInfo; these are not hard-coded fallback values.
Registered depth uses camera_color_optical_frame, matching RGB, and 16UC1
millimetres. The application converts depth to metres once.

## Checks

```bash
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /camera/depth/camera_info --once
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

Require positive focal lengths, matching dimensions/optical frames and fresh
timestamps. RGB and depth are associated within 35 ms by message timestamp; only the latest unprocessed
pair is retained. Hardware registration aligns pixels spatially; it does not imply
synchronized exposure times.

Stop any existing camera launcher and children before starting this stack.
Restarting only a shell wrapper can leave the ROS component container holding USB.
The new supervisor stops its whole process groups.

The original Gemini E launch file does not forward `enable_frame_sync`.
Testing it through the standalone node confirmed the SDK reports this device
does not support frame sync. It remains disabled explicitly. The 15 fps profile
and tighter software association reduce timestamp mismatch; they do not prove
hardware exposure synchronization. The application records both image timestamps
for subsequent timing analysis.
