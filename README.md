# Camera + Gyro

Local 6-DoF camera tracking, a coloured 3D point cloud, and a small web portal.
This branch replaces the old inertial/flow estimator and dashboard. The MI0802
USB thermal driver is retained; the Orbbec driver stays in its own workspace.

## Inputs and outputs

- RGB + registered depth estimate camera position and orientation in metres.
- MPU6050 **gyroscope only** supplies timestamped angular rates. Its accelerometer
  is never read. Bias is calibrated while stationary; rotation assistance is
  available only after mounting, scale and timing validation.
- MI0802 thermal imagery has its own portal tab. It is not assumed to align with RGB.
- The portal shows the camera frustum, trajectory, a bounded coloured point cloud,
  tracking health and the three camera feeds. Maps export as PLY.

The reference frame is the initial RGB optical frame: X right, Y down, Z forward.
The displayed pose maps current camera coordinates into that frame. It is not
gravity aligned. Visual odometry drifts; this is not a globally corrected SLAM map.
Lost/stale tracking retains the last accepted pose and stops adding scene points.
Return to a retained view to recover, or start a new map explicitly.

## Raspberry Pi setup

Ubuntu + ROS 2 Jazzy and the existing Orbbec workspace are required. Install the
Python packages from Ubuntu so cv_bridge and NumPy use compatible ABIs:

```bash
sudo apt install python3-numpy python3-opencv ros-jazzy-cv-bridge
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mi0802_senxor_driver
bash scripts/run.sh --host 0.0.0.0
```

Open **http://192.168.1.6:8080** for the current Pi. `--host 0.0.0.0` exposes
camera feeds and the map to the local network and was explicitly approved for
this setup. Omit it to bind only to Pi localhost; then on the Mac use:

```bash
ssh -N -L 8081:127.0.0.1:8080 andrew@192.168.1.6
```

Open `http://127.0.0.1:8081`. No cloud service or internet connection is needed
at runtime. Three.js and orbit controls are bundled with their MIT licence.
Ctrl+C stops the launched processes; logs are in `log/`.

The verified Gemini E profile is **RGB 640×360 + depth 640×360, 15 fps, hardware
depth-to-colour registration**. The 640×480 pair streams images but reports zero
factory intrinsics. See [camera setup](DEPTH_CAMERA_DRIVER_SETUP.md).

## Gyro calibration

Keep the module stationary for three seconds at startup. The portal distinguishes
Calibrated · not fused from Assisting. A stable bias does not prove rotational
scale, mounting or camera timestamp alignment. To enable assistance after testing
known rotations, pass `--gyro-config config/gyro.json`; use this structure:

```json
{
  "enabled": false,
  "mounting_validated": false,
  "rotation_camera_from_gyro": [[1,0,0],[0,1,0],[0,0,1]],
  "range_dps": 500,
  "scale": 1.0,
  "time_offset": 0.0
}
```

Replace the identity with the measured mount rotation and validate positive and
negative rotations on every axis before setting both flags true. Time offset is
added to gyro timestamps. Gaps and stale hardware invalidate rotation assistance.
There is no acceleration-based fallback.

## Development and evaluation on the Mac

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m unittest discover -s tests
python tests/test_pipeline.py
python scripts/evaluate.py
python -m tracking.server --demo
```

Demo mode raycasts a synthetic textured room through the real estimator. Its
images are labelled SIMULATION; it never substitutes for missing live data.
PnP is the default, chosen from comparisons on the actual Pi and camera.
Use `--method svd` or `--method pnp` to compare solvers. The browser renders the
3D scene; sensor processing runs on the Pi, offline evaluations on the Mac.

Record sequence saves up to 300 paired RGB-D frames in `recordings/` on the Pi.
New recordings include both image timestamps, gyro history and its bias/scale/time
metadata, and the estimated pose/status (not ground truth). Raw NPZ avoids costly
Pi compression; 300 frames use approximately 0.5 GB. Incomplete writes keep a
`.part` suffix and do not appear as completed frames. Stop early to limit storage. Evaluate a saved sequence with
`python scripts/evaluate.py recordings/<sequence>`; without external ground truth
this establishes throughput and repeatability, not pose accuracy.
For a brief Pi comparison without writing images, run
`python3 scripts/live_compare.py` with the camera running and ROS sourced.
For a downloaded TUM freiburg1_xyz dataset, run
`python scripts/evaluate_tum.py /path/to/rgbd_dataset_freiburg1_xyz --stride 6`.
Dataset files and recordings are excluded from Git.

## Validation (2026-09-08)

The 20 runtime unit tests and the pipeline check pass on both the Mac and Pi
(OpenCV 5.0 / NumPy 2.5 on Mac; OpenCV 4.6 / NumPy 1.26 on Pi).
Three additional offline calibration checks pass on the Mac; run calibration there.
The retained thermal driver's Pi colcon run passes all 6 reported tests.

Solver comparison on the Mac after the movement fixes; position RMSE includes held poses on lost
frames, with only the initial camera frame used for alignment:

| Input | SVD tracked | PnP tracked | SVD position RMSE | PnP position RMSE |
|---|---:|---:|---:|---:|
| Synthetic room, 80 frames | 79 | 79 | 0.0023 m | 0.0471 m |
| TUM fr1_xyz, stride 6, 133 frames | 126 | 127 | 0.0508 m | 0.0744 m |

[TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats) is a different camera and scene. These results do not establish accuracy
on this module. The initial stationary Pi comparison favored PnP; movement
accuracy and gyro mounting/timing still require physical motion validation.
No model training is required for either solver. Large dataset evaluation or
future training belongs on the Mac.

A 20-frame recording from this Gemini E was also evaluated on the Mac:
PnP tracked 19/20 frames (one initialization), median 6.1 ms, maximum
position variation 0.0008 m; SVD tracked 19/20, median 8.3 ms, variation
0.0080 m. This is a stationary repeatability check, not ground-truth accuracy.
The deployed portal passed live RGB/depth/thermal JPEG checks, PLY export,
map reset, bounded recording, and a clean supervisor stop/restart. Browser
checks covered the three live tabs, camera-follow, scene fit and no console
errors. The 20-frame recording remains on the Pi in
`recordings/20260908-141011`; its Mac evaluation copy is in
`/tmp/camera-gyro-live-recording`.

The Pi's isolated replay of that same recording takes a median 76.3 ms per
PnP frame (95th percentile 76.8 ms); SVD takes 106.3 ms. Replacing per-feature
NumPy median allocations with a nine-sample scalar median and skipping
unused current-depth sampling in PnP preserves the replay poses to within
3e-8 per transform element on the Mac. Edge/hole median equivalence is tested.

The earlier 5 fps live portal check processed 3.36 paired frames/s with median
104 ms processing time, maximum 118.5 ms, and 1.2 mm maximum stationary
position variation over 12 seconds. All three feeds and PLY export passed;
gyro bias was calibrated with zero I/O errors and rotation assistance disabled
pending physical mount validation. Pairing and camera timing limit throughput
below the configured 5 fps. This is a short stationary check, not a flight test.

### Movement recording follow-up

The user's 86-frame movement sequence exposed failures missed by stationary
checks: baseline PnP tracked 62/86 frames and some accepted poses disagreed
with measured depth by 20–25 cm. The recording also had gaps up to 2.1 s.
A stationary repeatability result must not be read as movement accuracy.

The runtime now captures at 15 fps and accepts RGB/depth message timestamps
at most 35 ms apart, retaining only the newest unprocessed pair. This Gemini E
rejects the SDK's frame-sync feature, so this is software timestamp association,
not synchronized exposure. Thermal preview work runs on the processing worker,
keeping ROS callbacks free to receive camera frames. Voxel keys are computed
in a batch; the Pi loop benchmark improved from 322 ms to 86 ms with identical
map output. Recording without compression measured 15 ms versus 112 ms per frame.
An initial 20 ms association cutoff caused periodic gaps up to 2.8 s as the two
unsynchronized streams drifted in phase. The 35 ms limit covers a 15 Hz
half-period; it still cannot correct motion between the two exposure times.

PnP now checks its accepted image matches against current depth and can recover
against the most recent accepted view. On the old movement recording it accepts
30/86 frames, versus 62/86 before: tracking continuity remains inadequate. Across
adjacent accepted pairs, median depth disagreement drops from 20 mm to 2.4 mm and
the worst from 245 mm to 65 mm, but the compared subsets differ. This measures
depth consistency, not ground-truth accuracy or successful reconstruction.
New movement data is required to assess the faster capture pipeline. Gyro
assistance remains disabled pending physical mounting and timing validation.

For new recordings containing gyro metadata, run
`python scripts/calibrate_gyro.py recordings/<sequence>` on the Mac. It estimates
a candidate mount rotation, scale and time offset against independent visual
rotations, and reports held-out residuals and timing ambiguity. Its output
always leaves assistance disabled; fit success alone is not physical validation.

After deployment, a 25-second stationary portal check processed 5.60 paired
frames/s, with median processing 145.9 ms and maximum observed frame age 182 ms.
RGB/depth/thermal JPEGs, the scene buffer and PLY export passed. A new 12-frame
stationary recording (`20260908-154823`) contains valid gyro histories and bias
metadata, no partial files, and a maximum frame interval of 336 ms. These checks
verify the capture repair, not movement-map quality.
