# Odometry node

`odom_node` consumes IMU samples from `/imu/data_raw`, estimates gyro bias and a gravity reference
while the drone is stationary, and publishes relative pose and velocity as
`nav_msgs/msg/Odometry` on `/odom`. It also broadcasts `odom -> base_link` unless `publish_tf` is
disabled.

To inspect the stationary IMU calibration independently, run the included Python sampler while
the robot is completely still. It collects exactly 1,000 valid samples by default and reports the
raw acceleration mean and noise, measured gravity magnitude, odometry acceleration scale, and
gyro bias and noise:

```bash
ros2 run odom_node imu_calibration_getter.py
```

Use `--topic`, `--samples`, or `--timeout` to override its defaults. For example:

```bash
ros2 run odom_node imu_calibration_getter.py --samples 1000 --timeout 30
```

To use the measurements, hand-fill `saved_raw_gyro_bias_rad_s` from `gyro_bias_rad_s`,
`saved_raw_acceleration_mean_m_s2` from `acceleration_mean_m_s2`, and
`saved_acceleration_scale_factor` from `odom_acceleration_scale_factor` in `params.yaml`, then set
`use_saved_calibration: true`. With saved calibration enabled, the node starts immediately instead
of running its own startup sample window. Set it back to `false` whenever the IMU mounting or
sensor changes.

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
