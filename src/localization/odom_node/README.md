# Odometry node

`odom_node` consumes IMU samples from `/imu/data_raw`, PMW3901 samples from
`/optical_flow/raw`, and VL53L1X distance from `/range/down`. It estimates gyro bias and a gravity
reference while the drone is stationary, then publishes relative pose and velocity as
`nav_msgs/msg/Odometry` on `/odom`. It also broadcasts `odom -> base_link` unless `publish_tf` is
disabled.

Optical flow is quality-gated and converted from counts to planar velocity using the live range
measurement. Samples are rejected when range is stale or invalid, flow quality is low, the shutter
indicates a dark surface, inferred speed is implausible, or the vehicle is rotating too fast.
Accepted velocity is blended with inertial velocity. The rangefinder provides tilt-compensated
relative Z position and vertical velocity; its first valid sample after calibration defines Z=0
unless `range_reference_distance_m` is configured. Large range innovations are rejected so a
sudden return from furniture or another non-floor surface cannot immediately jump odometry.

The initial `flow_radians_per_count: 0.0025` and `flow_to_body_matrix` are calibration values, not
universal properties of every PMW3901 lens and mounting. At a fixed measured height, translate the
drone forward without rotating it and confirm `/odom.twist.twist.linear.x` is positive; translate
left and confirm Y is positive. Change matrix signs/order if necessary. Then compare a measured
translation or velocity with odometry and scale `flow_radians_per_count` proportionally. Do this
before flight. Optical flow observes velocity, not absolute XY position, so it reduces IMU
velocity drift but cannot eliminate accumulated position error by itself.

Both gyro and acceleration pass through a rolling mean before calibration, attitude integration,
translation integration, and calibrated IMU publication. `imu_average_window_size: 10` uses the
latest ten reads while retaining the input update rate after the window fills. At 100 Hz this adds
about 45 ms of mean measurement delay; set the parameter to `1` to disable averaging.

To inspect the stationary IMU calibration independently, run the included Python sampler while
the robot is completely still. It collects exactly 1,000 valid samples by default and reports the
raw acceleration mean and noise, measured gravity magnitude, odometry acceleration scale, and
gyro bias and noise:

```bash
ros2 run odom_node imu_calibration_getter.py
```

The MPU driver must already be publishing. Check it with `ros2 topic info /imu/data_raw` and
`ros2 topic hz /imu/data_raw`. If no publisher exists, start the IMU-only graph in another sourced
terminal with `ros2 launch drone_control drone_launch.py start_imu:=true start_odom:=false`.
Do not start a second MPU driver when the dashboard launch already owns the I2C device.

Use `--topic`, `--samples`, or `--timeout` to override its defaults. For example:

```bash
ros2 run odom_node imu_calibration_getter.py --samples 1000 --timeout 30
```

The getter is diagnostic only. `odom_node` performs its own stationary calibration automatically
on every startup, learning current gyro bias, gravity direction, and accelerometer scale before it
publishes odometry.

Calibration also learns a leveling rotation from the stationary gravity direction. The node
applies that rotation consistently to acceleration and gyro axes, so a stationary calibrated IMU
reports approximately `[0, 0, +9.80665] m/s^2` even when the physical sensor is mounted at an
angle. The sign follows `sensor_msgs/Imu`: an accelerometer measures specific force, so at rest
it reads `+g` along the frame's up axis, not `-g`. Relative odometry orientation still starts at
identity; yaw remains defined by the startup heading because gravity cannot observe yaw.

Levelling therefore corrects tilt only. It cannot correct how the board is rotated *about*
gravity, so an IMU that is not mounted with its `+X` forward and `+Y` left needs that mounting
rotation declared in `imu_to_body_rotation_rpy`; otherwise roll, pitch, and yaw come out swapped
or mirrored no matter how well the node is calibrated.

With the supplied `integrate_linear_acceleration: true` configuration, the stationary calibration
also learns an acceleration scale that maps the measured gravity magnitude to `9.80665 m/s^2`.
This handles MPU-compatible boards whose effective range differs from register readback. Set
`auto_scale_acceleration: false` only when the IMU's SI scale is already known to be accurate.
Mounted acceleration is then rotated into `odom`, the calibrated gravity reference is removed,
and the remainder is lightly low-pass filtered and integrated in three dimensions. Deadbanding,
velocity damping, and acceleration and speed limits constrain obvious runaway. The node does not
infer stationarity or apply zero-velocity updates from IMU data because steady motion is
indistinguishable from rest to an IMU. This makes existing `/odom` clients react to linear motion
without app changes. When flow or range is rejected, the estimator falls back to inertial dead
reckoning; small bias and attitude errors are then integrated twice. Position must not be treated
as safety-grade or a long-term absolute estimate. Add VIO, GPS, or another absolute reference for
bounded global XY error. Set `integrate_linear_acceleration: false` to use accepted flow for planar
translation without the inertial translation fallback.

For stationary bench testing, set `static_override: true` or launch with
`static_override:=true`. This skips gyro calibration and integration and publishes a fixed identity
pose at the `odom` origin with `static_position_variance`. The fixed pose is advertised as
calibrated and position-observed, so the iPhone app reports **Robot track: Tracking**. Disable the
override and restart before the robot can move.

Set `quality_override: true` (or launch with `quality_override:=true`) to force
`quality_override_position_variance` regardless of the inertial estimator's growing uncertainty.
It changes only advertised confidence, not the pose estimate or its drift.

The node rotates IMU samples from the mounted sensor frame into `base_link`, removes stationary
gyro bias, and integrates the midpoint of consecutive angular-rate samples. Translation uses the
same published orientation seen by clients. The node does not integrate across out-of-order
timestamps or gaps longer than `max_imu_gap_sec`.

Build and run it directly:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to odom_node
source install/setup.bash
ros2 launch odom_node odom_launch.py
```

Or run it as part of the drone graph:

```bash
ros2 launch drone_control drone_launch.py start_odom:=true
```

The equivalent top-level bench launch is:

```bash
ros2 launch drone_control drone_launch.py \
  start_odom:=true odom_static_override:=true
```

To keep live gyro orientation but force good reported position quality instead:

```bash
ros2 launch drone_control drone_launch.py \
  start_odom:=true odom_quality_override:=true
```

Keep the drone stationary while the first `startup_initialization_samples` messages arrive. Gyro
sample variation rejects a window with movement. Accelerometer variation produces a warning but
does not block gyro calibration; its window mean establishes the gravity reference. If that mean
is outside the gravity sanity range, the node continues with orientation odometry and disables
inertial position integration instead of remaining uncalibrated. Calibration state is
transient-local on `/odom/calibrated` and is also republished once per second so rosbridge clients
that connect after startup still receive it. Recalibration resets pose and velocity to the odom
origin.

Request recalibration while the drone is stationary:

```bash
ros2 service call /odom/calibrate std_srvs/srv/Trigger '{}'
```

`/imu/data_calibrated` contains the integrated relative orientation, bias-corrected angular
velocity, and mounted-frame acceleration with covariance. Acceleration includes gravity and is
published for diagnostics; the gravity-compensated value is integrated internally when
`integrate_linear_acceleration` is enabled.
Gyro-only orientation has no absolute heading or gravity reference and will drift over time.
Calibration requests are rejected while the static override is active.
