# Calibration

This project has **three independent calibrations**, one per sensor pair.
They use different methods and have different levels of completeness:

| # | Pair | Method | Status |
|---|------|--------|--------|
| 1 | ZED 2 stereo intrinsics | Read directly from ZED SDK/firmware | Done — `calibration/zed_intrinsics.yaml` |
| 2 | Camera ↔ MTi-8 IMU extrinsic | Kalibr (AprilGrid + motion capture) | Pipeline exists; run per-rig |
| 3 | Radar (AWR1843) ↔ Camera extrinsic | Manual ruler/caliper measurement | Done (translation only) — `config/settings.py` |

`camera_to_vehicle.yaml` and `gnss_to_vehicle.yaml` are written as
placeholder identity transforms by `dataset/writer.py` but are not
currently measured or read by any code — they exist as slots for a
future full sensor-to-vehicle calibration.

---

## 1. ZED 2 Stereo Intrinsics

The ZED 2 factory-calibrates itself; there is nothing to physically
calibrate here — the scripts just extract and freeze the SDK's numbers so
the rest of the pipeline (radar/camera overlay, Kalibr camchain, dataset
metadata) uses one fixed, versioned copy instead of re-querying the SDK.

**Script:** `calibration/export_zed_calibration.py`

```bash
python calibration/export_zed_calibration.py
```

It opens the camera, reads both the **rectified** and **raw** (unrectified)
calibration parameters plus the stereo extrinsic transform from
`camera_information().calibration_parameters`, and writes one immutable
file: `calibration/zed_intrinsics.yaml`. Contents:

- `rectified.left` / `rectified.right` — `fx, fy, cx, cy`, distortion,
  FOV, focal length in mm, per rectified eye.
- `raw.left` / `raw.right` — same fields, before rectification.
- `stereo_rotation`, `stereo_translation`, `baseline_m` — extrinsic
  between the two eyes.
- `projection_matrix.left/right`, `rectification_matrix.left/right` —
  standard stereo-rectified projection/rectification matrices (right's
  projection matrix bakes in `-fx * baseline` for the horizontal shift).

This file is what `ZEDCamera.open()` loads into `camera.calib`, which is
what `radar/projection.py` and `dashboard/radar_panel.py` use for `fx, fy,
cx, cy` when projecting radar points onto the image, and what
`convert_for_kalibr.py` uses to seed the Kalibr `camchain.yaml`.

**Validation script:** `calibration/validate_zed_calibration.py`

```bash
python calibration/validate_zed_calibration.py
```

Grabs one live stereo frame, computes depth manually from disparity as
`Z = fx * baseline / disparity` using the exported `zed_intrinsics.yaml`
values, and compares it pixel-by-pixel against the ZED SDK's own depth
map (`sl.MEASURE.DEPTH`) at 1000 sampled points. It reports mean/median/
max error and RMSE, writes a red/blue error heatmap next to the left
image (`calibration/validation_error.png`), and prints PASS if mean error
< 2 cm and RMSE < 3 cm — this just confirms the exported YAML is
internally consistent with what the SDK is actually using, not that the
camera itself is well calibrated.

**Re-run when:** the ZED 2 undergoes a firmware update, or you swap to a
different physical unit (a new serial number).

---

## 2. Camera ↔ MTi-8 IMU Extrinsic (Kalibr)

This is the calibration that estimates the rigid transform between the
ZED 2's optical frame and the MTi-8's body frame — needed for any
tightly-coupled visual-inertial fusion. It follows the standard
Kalibr `camera-imu` workflow: print a known target, record synchronized
motion, convert to Kalibr's input format, and run Kalibr's optimizer
externally.

### Coordinate frames (`calibration/coordinate_frames.md`)

- **ZED 2 optical frame:** X=right, Y=down, Z=forward (into the scene).
  This is the frame `zed_intrinsics.yaml` is expressed in.
- **MTi-8 body frame:** X=forward, Y=left, Z=up (default Xsens frame,
  label facing up). The MTi-8's quaternion output is body→NED.
- **Transform being solved for:** `T_camera←imu = [R | t]` such that
  `p_camera = R @ p_imu + t`. If the IMU is mounted flat below the camera
  as expected, `R` should be close to a fixed ~90°/90°/0° axis
  permutation (documented as a sanity check, not a substitute for
  calibrating).

### Step 1 — Print the target

```bash
python calibration/generate_aprilgrid.py
```

Generates a 6×6 AprilGrid (tag family `36h11`, 80 mm tags, 0.3× spacing
ratio → 600×600 mm board):

- `calibration/aprilgrid.pdf` — print at **100% scale**, no "fit to page".
- `calibration/april_6x6.yaml` — Kalibr target config (tag layout/size).
- `calibration/aprilgrid_preview.png` — quick visual check.

Mount the printout on a rigid, flat surface — curling or a non-rigid
backing corrupts every detected corner.

### Step 2 — Measure the rig (`calibration/rig_measurements.md`)

Before recording, fill in this checklist by hand:

- **d_forward / d_height / d_lateral** — the IMU-to-camera offset along
  each camera axis (Z/Y/X respectively), used as a sanity check against
  Kalibr's estimated `t`, not fed into Kalibr directly.
- **MTi-8 mounting orientation** (flat / upside-down / rotated by how
  many degrees) and **mounting method** (rigid bracket vs. anything that
  can flex — flexible mounts invalidate the calibration since the
  solved transform is assumed constant).
- **Target checklist** — AprilGrid printed at correct scale, tag size
  verified with a ruler (80 mm), board flat and rigidly mounted.

This file is a record to keep with the calibration output, not an input
any script reads.

### Step 3 — Record calibration motion

```bash
python calibration/record_calibration.py --duration 180
```

Opens the ZED and starts the MTi-8 receiver, then records for the given
duration (default 3 minutes) into `calibration/dataset/` using the same
`DatasetWriter`/`Synchronizer` machinery as normal data collection. The
IMU is sampled at its own native rate (200 Hz) independent of camera
frame timing — `synchronize_dataset.py`'s offline pairing (see
`SYNCHRONIZATION.md`) is what later builds the matched camera/IMU pairs
Kalibr needs, not this script.

While recording, move the rig in front of the AprilGrid to excite all 6
degrees of freedom:

- Slow translations along X, Y, Z
- Rotations: roll, pitch, yaw
- Figure-eight and circular motions
- Gentle tilting, keeping the target in frame at 0.5–3 m

Avoid: violent shaking (motion blur breaks corner detection), long
stationary stretches (no new information for the optimizer).

### Step 4 — Convert to Kalibr's input format

```bash
python calibration/convert_for_kalibr.py calibration/dataset --bag calibration.bag
```

Reads the recorded dataset (auto-detects v1 CSV/image or v2
Parquet/HDF5 format via `dataset/reader.py`) and produces, under
`calibration/kalibr_input/` by default:

- `cam0/`, `cam1/` — left/right frames renamed to their nanosecond
  timestamp (`<ts_ns>.png`).
- `imu0.csv` — IMU samples as `timestamp, omega_x/y/z, alpha_x/y/z`.
- `imu.yaml` — MTi-8 noise parameters (accelerometer/gyroscope noise
  density and random walk, hardcoded from the datasheet) plus
  `update_rate: 200.0`.
- `camchain.yaml` — seeded with `fx, fy, cx, cy` from
  `zed_intrinsics.yaml` (or 1280×720 hardcoded fallback values if no
  calibration file is passed), zero distortion (rectified images already
  have distortion removed), the stereo baseline as `T_cn_cnm1`, and a
  **placeholder** `T_cam_imu` rotation (a 90°/90° axis swap matching the
  expected mounting from `coordinate_frames.md`) — this is Kalibr's
  *initial guess*, not the calibrated answer; Kalibr overwrites it.

With `--bag`, it also serializes everything into a ROS1 `.bag` file
(via the pure-Python `rosbags` library — no ROS install needed) with
topics `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`, matching what
`kalibr_calibrate_imu_camera` expects.

### Step 5 — Run Kalibr (external tool)

Kalibr itself isn't part of this repo; run it separately (Docker image or
ROS install) on the outputs of step 4:

```bash
kalibr_calibrate_imu_camera \
  --bag calibration.bag \
  --cam calibration/kalibr_input/camchain.yaml \
  --imu calibration/kalibr_input/imu.yaml \
  --target calibration/april_6x6.yaml \
  --imu-models calibrated
```

Kalibr detects the AprilGrid in every frame, jointly optimizes camera
intrinsics/extrinsics and the camera↔IMU transform against the IMU's
inertial measurements, and outputs a report plus an updated
`camchain-imucam-*.yaml` containing the calibrated `T_cam_imu`. That's
the number that answers "how is the IMU actually mounted relative to the
camera" — compare its rotation against the `coordinate_frames.md` sanity
table and its translation against the `rig_measurements.md` ruler
measurements as a gut check before trusting it.

**Re-run when:** the physical mount between the ZED 2 and MTi-8 changes
in any way (re-bracketed, re-tightened, replaced) — the whole premise of
this calibration is that the relative pose never moves after it's solved.

---

## 3. Radar (AWR1843) ↔ Camera Extrinsic

Unlike the camera-IMU pair, this one is a simple manual measurement, not
an optimization — documented in full in `docs/RADAR_CAMERA_OVERLAY.md`.

**Where it's used:** `radar/projection.py`'s `project_point()`, shared by
`radar_camera_demo.py` and `dashboard/radar_panel.py`'s
`RadarCameraOverlay`, to draw radar detections on the camera image:

```python
dx, dy, dz = config.settings.RADAR_TO_CAMERA_OFFSET_M   # radar->camera lever arm, radar frame
Xc, Yc, Zc = (p.x - dx), -(p.z - dz), (p.y - dy)         # -> pinhole (x=right, y=down, z=forward)
u = cx + fx * Xc / Zc
v = cy + fy * Yc / Zc
```

`fx, fy, cx, cy` come from the ZED's exported calibration (part 1 above).
`RADAR_TO_CAMERA_OFFSET_M` is a **translation-only** lever arm (no
rotation term — the two sensors are assumed mounted parallel/boresight-
aligned) currently set to `(-0.055, 0.004, -0.015)` m in
`config/settings.py`.

**How it was measured:** with calipers/ruler, from the AWR1843's antenna
array phase center (the tan/gold patch-antenna region on the BOOST
board — not the board edge or SMA connector) to the ZED's **left** lens
optical center, expressed in the radar's own frame (x=right, y=forward,
z=up):

- `dx` — left/right offset, positive if the camera lens is to the
  radar's right.
- `dy` — forward/back offset, positive if the camera lens is ahead of
  (closer to the scene than) the radar.
- `dz` — up/down offset, positive if the camera lens is higher than the
  radar.

`dy`'s sign in the current value is an assumption from a rig photo
rather than an independent measurement (its magnitude is small enough
not to matter much beyond a meter or so).

**Re-measure when:** the radar or camera is remounted — this value is
only valid for the exact geometry it was measured on, and there's no
optimizer here to catch drift the way Kalibr would.

**Known limitation:** no rotation term exists yet. If the two sensors
stop being boresight-parallel, the current projection math would need a
rotation matrix added the same way `ti_mmwave_rospkg`'s reference
implementation uses a full 6-DOF static transform (see
`docs/RADAR_CAMERA_OVERLAY.md` for that reference).

---

## 4. Vehicle-frame sensor geometry

`fusion/transforms.py` + `calibration/extrinsics/extrinsics.yaml` hold the
pose of every sensor (camera, radar, lidar, GNSS) in one vehicle frame:
origin at the MTi-8 IMU, x forward, y left, z up. Each entry is a
translation plus optional mount tilt (`rpy_deg`) on top of the sensor's
native axes (radar x-right/y-fwd/z-up, ZED optical x-right/y-down/z-fwd,
Ouster/IMU x-fwd/y-left/z-up).

```python
from fusion.transforms import load_extrinsics
ext = load_extrinsics()                       # warns about placeholders
p_cam = ext.transform(pts_radar, "radar", "camera")
p_veh = ext.transform(pts_lidar, "lidar", "vehicle")
```

`radar/projection.py:project_points_se3()` does the full 6-DoF radar→pixel
projection. **Status:** only the radar↔camera relative offset is measured;
all absolute positions are placeholders (`measured: false`) until the rig
is measured — `load_extrinsics(strict=True)` refuses them. A test keeps the
file consistent with `RADAR_TO_CAMERA_OFFSET_M`.

---

## Where results are stored in a recorded dataset

Every dataset written by `DatasetWriter` gets a `calibration/` folder
containing:

- The ZED calibration active at record time (`writer.save_calibration()`,
  called from `record_calibration.py` and normal recording scripts).
- `calibration/extrinsics/camera_to_mti.yaml`, `camera_to_vehicle.yaml`,
  `gnss_to_vehicle.yaml` — written as **identity-transform placeholders**
  if they don't already exist. Nothing in the codebase currently reads
  these back in (`validate_dataset.py` and `dataset/writer.py` only
  check for/copy their presence) — they're reserved slots for the
  Kalibr `T_cam_imu` result (part 2) and a future full vehicle-frame
  calibration, not yet wired into any consumer.
- `calibration/extrinsics/radar_to_camera.yaml` — the only one of these
  four that's real: rewritten on **every** recording (not just when
  missing) from the live `RADAR_TO_CAMERA_OFFSET_M`, so each dataset
  records the offset actually in effect when it was captured.

---

## Extrinsic solvers (lidar, radar) — DE global search + least-squares polish

`calibration/extrinsic_solvers.py` solves ONE sensor's pose in the vehicle
frame in the same parametrisation as `extrinsics.yaml`, holding the others
fixed. Each solve runs differential evolution inside tape-measure bounds from
several seeds, polishes each with Levenberg-Marquardt, then reports the best
pose, the seed-to-seed spread, a Jacobian 1-sigma, and any parameter pinned at
its bound. Large spread or a parameter at a bound means the data do not
constrain that DOF — fix it (`fixed=(...)`) or collect better poses.

| Pair | Function | Data needed |
|------|----------|-------------|
| Lidar -> camera (target) | `solve_lidar_camera_correspondences` | 3-D lidar points (cutout-board hole centres/corners) + matching pixels, >= 6, many board poses 3-12 m |
| Lidar -> camera (targetless) | `solve_lidar_camera_nid` | (lidar points, intensity, gray image) from city drives; lane markings/manholes give the edges |
| Radar -> lidar | `solve_radar_lidar` | corner-reflector positions in both sensors (>= 4, varied range/azimuth); residuals in radar polar space |
| Radar yaw | `solve_radar_yaw_from_doppler` | per-frame radial velocities + reference forward speed |

CLI for the correspondence solvers (writes the yaml only when the result is
trusted):

```bash
python calibration/solve_extrinsics.py lidar-camera --data board.npz --write
python calibration/solve_extrinsics.py radar-lidar  --data refl.npz --range 0.2 0.2 0 0 0 8 --write
```

Not implemented: detecting the board holes/reflectors in the pcap, SVO2 and
radar data to produce those `.npz` correspondences, and the MTi/GNSS-to-lidar
hand-eye (use lidar odometry + a Kalibr/hand-eye tool for that pair).
