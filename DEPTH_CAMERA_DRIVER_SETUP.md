# Gemini E camera profile

The installed legacy Orbbec ROS 2 main driver works with this Gemini E
(wrapper 1.5.15, SDK 1.10.35). Keep it in ~/orbbec_ws; verify legacy-device
compatibility before changing driver branches.

## Verified live profile

```bash
source /opt/ros/jazzy/setup.bash
source ~/orbbec_ws/install/setup.bash
ros2 launch orbbec_camera gemini_e.launch.py enable_color:=true color_width:=640 color_height:=360 color_fps:=5 enable_depth:=true depth_width:=640 depth_height:=360 depth_fps:=5 enable_ir:=false depth_registration:=true align_mode:=HW enable_point_cloud:=false
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
timestamps. RGB and depth are associated within 65 ms; only the latest unprocessed
pair is retained. Hardware registration aligns pixels spatially; it does not imply
synchronized exposure times.

Stop any existing camera launcher and children before starting this stack.
Restarting only a shell wrapper can leave the ROS component container holding USB.
The new supervisor stops its whole process groups.
