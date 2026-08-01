# VIO node

`vio_node` combines raw IMU propagation with sparse monocular visual motion and publishes
`nav_msgs/msg/Odometry` on `/vio/odometry`. It also broadcasts `odom -> base_link` unless
`publish_tf` is disabled. Calibration state is latched on `/vio/calibrated` so observers do not
miss the completion event.

With `calibrate_on_startup: true` (the default), the node automatically waits for a stationary
IMU initialization window whenever it starts. It
tracks Shi-Tomasi corners with pyramidal Lucas-Kanade optical flow, rejects outliers with an
essential-matrix RANSAC, corrects the propagated attitude from visual rotation, and uses the
IMU-propagated displacement to resolve the monocular translation scale.

On the Raspberry Pi, visual tracking is limited to `visual_processing_rate_hz` and images are
resized by `image_processing_scale`. The defaults process the 30 FPS color stream at 5 FPS and
half resolution so optical flow does not starve 100 Hz IMU propagation.

Build and run it directly:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to vio_node
source install/setup.bash
ros2 launch vio_node vio_launch.py
```

Or run it as part of the drone graph:

```bash
ros2 launch drone_control drone_launch.py \
  start_depth_camera:=true start_vio:=true
```

Keep the drone stationary while the first `startup_initialization_samples` IMU messages are
received. The startup default is 100 samples; manual `/vio/calibrate` requests continue to use
the faster 20-sample `initialization_samples` window.
Because calibration is only requested while stationary, every finite IMU message counts toward
the sample window. The mean angular rate supplies gyro bias, while the stationary acceleration
direction supplies initial roll/pitch and its magnitude supplies a scalar accelerometer scale.
A single stationary pose cannot independently identify three-axis accelerometer bias.

After calibration, `/imu/data_calibrated` contains bias-corrected angular velocity and
current-attitude gravity-compensated linear acceleration. Both are approximately zero while the
drone is stationary at any orientation; normal sample noise inside the configured deadbands is
published as exactly zero. `/imu/data_raw` and the samples used internally by VIO are unchanged.

`/vio/odometry` is always propagated by the IMU, so its presence alone does not prove the camera
path is working. `/vio/visual_tracking` is `true` only when the most recently processed image
produced an accepted essential-matrix update. Accepted visual updates are also logged at INFO.
Calibrate the camera and measure both sensor-to-body rotations before flight. Monocular VIO has
no independent visual scale; poor accelerometer bias or a moving initialization will therefore
produce poor metric translation even when visual attitude tracking looks healthy.

Request a full VIO recalibration while the drone is stationary:

```bash
ros2 service call /vio/calibrate std_srvs/srv/Trigger '{}'
```

The service returns immediately after resetting the odometry origin and visual tracker. It then
re-estimates gyro bias, accelerometer scale, and gravity alignment during a stationary sample
window. `/vio/calibrated` changes to `false` during calibration and back to `true` when the active
sample window is complete. Completion and the estimated biases are also reported in the log.
