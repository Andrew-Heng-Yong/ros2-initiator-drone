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

The verified Gemini E profile is **RGB 640×360 + depth 640×360, 5 fps, hardware
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
Stop it early to limit storage. Evaluate a saved sequence with
`python scripts/evaluate.py recordings/<sequence>`; without external ground truth
this establishes throughput and repeatability, not pose accuracy.
For a brief Pi comparison without writing images, run
`python3 scripts/live_compare.py` with the camera running and ROS sourced.
For a downloaded TUM freiburg1_xyz dataset, run
`python scripts/evaluate_tum.py /path/to/rgbd_dataset_freiburg1_xyz --stride 6`.
Dataset files and recordings are excluded from Git.

## Validation (2026-09-08)

The 16 Python tests and the pipeline check pass on both the Mac and Pi
(OpenCV 5.0 / NumPy 2.5 on Mac; OpenCV 4.6 / NumPy 1.26 on Pi).
The retained thermal driver's Pi colcon run passes all 6 reported tests.

Final solver comparison on the Mac; position RMSE includes held poses on lost
frames, with only the initial camera frame used for alignment:

| Input | SVD tracked | PnP tracked | SVD position RMSE | PnP position RMSE |
|---|---:|---:|---:|---:|
| Synthetic room, 80 frames | 79 | 79 | 0.0023 m | 0.0471 m |
| TUM fr1_xyz, stride 6, 133 frames | 126 | 128 | 0.0508 m | 0.0872 m |

[TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats) is a different camera and scene. These results do not establish accuracy
on this module. The initial stationary Pi comparison favored PnP; movement
accuracy and gyro mounting/timing still require physical motion validation.
No model training is required for either solver. Large dataset evaluation or
future training belongs on the Mac.

A 20-frame recording from this Gemini E was also evaluated on the Mac:
PnP tracked 19/20 frames (one initialization), median 17.6 ms, maximum
position variation 0.0008 m; SVD tracked 19/20, median 17.9 ms, variation
0.0080 m. This is a stationary repeatability check, not ground-truth accuracy.
The deployed portal passed live RGB/depth/thermal JPEG checks, PLY export,
map reset, bounded recording, and a clean supervisor stop/restart. Browser
checks covered the three live tabs, camera-follow, scene fit and no console
errors. The 20-frame recording remains on the Pi in
`recordings/20260908-141011`; its Mac evaluation copy is in
`/tmp/camera-gyro-live-recording`.
