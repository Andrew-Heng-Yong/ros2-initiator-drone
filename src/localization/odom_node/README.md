# Odometry node

`odom_node` consumes gyroscope samples from `/imu/data_raw`, estimates startup bias while the
drone is stationary, and publishes relative orientation and angular velocity as
`nav_msgs/msg/Odometry` on `/odom`. It also broadcasts `odom -> base_link` unless `publish_tf`
is disabled.

This interim estimator intentionally ignores images and linear acceleration. Position and linear
velocity remain zero with a large covariance because they are unobserved. Flow-sensor updates can
later supply translation without changing the gyro propagation boundary.

For stationary bench testing, set `static_override: true` or launch with
`static_override:=true`. This skips gyro calibration and integration and publishes a fixed identity
pose at the `odom` origin with `static_position_variance`. The fixed pose is advertised as
calibrated and position-observed, so the iPhone app reports **Robot track: Tracking**. Disable the
override and restart before the robot can move.

The node rotates gyro samples from the mounted IMU frame into `base_link`, removes the stationary
bias, applies a small noise deadband, and integrates the midpoint of consecutive angular-rate
samples. It does not integrate across out-of-order timestamps or gaps longer than
`max_imu_gap_sec`.

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

Keep the drone stationary while the first `startup_initialization_samples` messages arrive. A
calibration window is rejected and restarted when its mean angular speed or sample standard
deviation indicates motion. Calibration state is transient-local on `/odom/calibrated` and is also
republished once per second so rosbridge clients that connect after startup still receive it.

Request recalibration while the drone is stationary:

```bash
ros2 service call /odom/calibrate std_srvs/srv/Trigger '{}'
```

`/imu/data_calibrated` contains the integrated relative orientation and bias-corrected angular
velocity. Its linear-acceleration covariance starts with `-1` to mark that field unavailable.
Gyro-only orientation has no absolute heading or gravity reference and will drift over time.
Calibration requests are rejected while the static override is active.
