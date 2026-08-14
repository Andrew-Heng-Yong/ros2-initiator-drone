# Odometry node

`odom_node` consumes IMU samples from `/imu/data_raw`, estimates gyro bias and a gravity reference
while the drone is stationary, and publishes relative pose and velocity as
`nav_msgs/msg/Odometry` on `/odom`. It also broadcasts `odom -> base_link` unless `publish_tf` is
disabled.

Both gyro and acceleration pass through a rolling mean before calibration, attitude integration,
translation integration, and calibrated IMU publication. `imu_average_window_size: 10` uses the
latest ten reads while retaining the input update rate after the window fills. At 100 Hz this adds
about 45 ms of mean measurement delay; set the parameter to `1` to disable averaging.

Stationarity uses low temporal variance in a window of gravity-compensated planar acceleration,
low gyro rate, and a broad mean-bias sanity cap. It does not require acceleration bias to fall
inside a small fixed deadband. Once a stationary interval is confirmed, the node zeros velocity,
adapts its gravity reference, and restores the position from the beginning of the confirmation
window so persistent sensor bias during that window is not recorded as displacement.

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
reports approximately `[0, 0, -9.80665] m/s^2` even when the physical sensor is mounted at an
angle. Relative odometry orientation still starts at identity; yaw remains defined by the startup
heading because gravity cannot observe yaw.

With the supplied `integrate_linear_acceleration: true` configuration, the stationary calibration
also learns an acceleration scale that maps the measured gravity magnitude to `9.80665 m/s^2`.
This handles MPU-compatible boards whose effective range differs from register readback. Set
`auto_scale_acceleration: false` only when the IMU's SI scale is already known to be accurate.
Mounted acceleration is then rotated into `odom`, the calibrated gravity reference is removed,
and the remainder is integrated into velocity and position. Deadbanding, velocity damping and
limits, and a stationary zero-velocity update constrain obvious runaway.
`planar_translation: true` projects acceleration
onto the plane perpendicular to the calibrated gravity vector, maps that plane onto odom X/Y, and
locks Z velocity and position to zero. This remains responsive when the physical IMU Z axis is not
the robot's vertical axis. It also prevents a small gravity error from integrating into a vertical
launch or fall. Disable it only when genuine vertical motion is required. The planar acceleration
is lightly low-pass filtered before integration; the configured deadband and stationary threshold
are deliberately below ordinary gentle robot acceleration. This makes existing `/odom` clients
react to linear motion without app changes. It is still IMU-only dead reckoning: small bias and
attitude errors are integrated twice, so X/Y position will drift and must not be treated as a
safety-grade or long-term position estimate. Add optical flow, VIO, wheel odometry, GPS, or another
external reference for reliable translation. Set `integrate_linear_acceleration: false` to restore
the orientation-only behavior and unobserved position covariance.

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
same published orientation seen by clients. A sample window is considered stationary only when
both angular rate and gravity-compensated acceleration remain near zero; sustained
stationarity zeros velocity and slowly adapts the gravity reference. The node does not integrate
across out-of-order timestamps or gaps longer than `max_imu_gap_sec`.

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
