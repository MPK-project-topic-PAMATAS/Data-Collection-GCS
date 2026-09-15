# Dataset Format v2

## Overview

Every sensor is recorded independently at its own native rate — see
`SYNCHRONIZATION.md` for the full architecture. Synchronization is a
separate, later, offline step (`metadata/synchronization/`); it never gates
what reaches this format.

v2 replaces per-frame CSV/JSON with columnar formats:
- **Images**: SVO2 recording only (all ZED2 video and IMU; no per-frame images)
- **IMU/sensors**: MTi-8 HDF5 with chunked+lzf compression, daily rotation, at
  the MTi's native ~200 Hz rate
- **Frame index**: Partitioned Parquet (date-based), one row per camera frame
- **Radar/GNSS index**: Partitioned Parquet (date-based), one row per native
  radar frame / GNSS fix
- **Synchronization index**: Partitioned Parquet (date-based), **derived**
  nearest-match + residuals, built offline (`synchronize-dataset` CLI)
- **Sessions/events**: Partitioned Parquet
- **Calibration**: YAML/JSON (unchanged)
- **Config snapshot**: JSON, one per session, immutable
- **Radar**: AWR1843 point clouds and range–Doppler heatmaps (TLV1/TLV5 only
  — raw ADC/IQ is not captured; see `DATASET_WRITER.md`)
- **Lidar**: Ouster OS-0-64, 64-channel — exactly one pcap + one config JSON
  per session, written directly by `lidar/receiver.py` at native rate; scans
  are decoded from the pcap offline, on demand, never pre-generated per-frame
- **GNSS**: raw NMEA text log + structured fix index, both at native rate

## Directory Structure

Every recording session gets its own subfolder under the dataset root,
named by its `session_id` — `DatasetWriter.begin_session()` creates it and
switches all writes into it. Each session's subfolder is a complete,
self-contained copy of the whole tree below; nothing is shared between two
sessions, and nothing outside the session's own subfolder is touched while
it's recording:

```
dataset/                             # the dataset root — never written to directly
└── <session_id>/                    # e.g. 20260728_143021 — one full tree per session
    ├── left/                          # (not written — video is in SVO2)
    ├── right/                         # (not written — video is in SVO2)
    ├── depth/                         # float32 NPY depth maps
    │   └── 20260728_143021_123456789_0000000042.npy
    ├── disparity/                     # float32 NPY disparity maps
    ├── confidence/                    # float32 NPY confidence maps
    ├── sensors/                       # HDF5 sensor data
    │   └── 20260728_000000_mti.h5     # daily rotation, scoped to this session
    ├── metadata/
    │   ├── frames/                    # partitioned Parquet, one row per camera frame
    │   │   └── date=20260728/
    │   │       ├── part-000001.parquet
    │   │       └── part-000002.parquet
    │   ├── radar/                     # partitioned Parquet, one row per native radar frame
    │   │   └── date=20260728/
    │   │       └── part-000001.parquet
    │   ├── gnss/                      # partitioned Parquet, one row per native GNSS fix
    │   │   └── date=20260728/
    │   │       └── part-000001.parquet
    │   ├── synchronization/           # DERIVED — built offline, see SYNCHRONIZATION.md
    │   │   └── date=20260728/
    │   │       └── part-000001.parquet
    │   ├── sessions/
    │   │   └── date=20260728/
    │   │       └── part-000001.parquet
    │   ├── events/
    │   │   └── date=20260728/
    │   │       └── part-000001.parquet
    │   ├── config_snapshot_20260728_143021.json   # this session's config, immutable
    │   └── session_manifest_20260728_143021.json  # this session's file paths + timing
    ├── calibration/
    │   ├── zed_intrinsics.yaml
    │   ├── camera_info.json
    │   └── extrinsics/
    │       ├── camera_to_mti.yaml
    │       ├── camera_to_vehicle.yaml
    │       └── gnss_to_vehicle.yaml
    ├── radar/
    │   ├── points/                    # JSON point cloud per native radar frame
    │   ├── heatmap/                   # uint16 NPY, shape: (range_bins, Doppler_bins)
    │   └── range_profile/             # uint16 NPY, shape: (range_bins,), when enabled
    ├── manifests/
    │   └── dataset_manifest.json
    └── raw/                           # SVO2, Xbus binary, lidar PCAP + JSON, radar bag, NMEA
        ├── recording.svo2
        ├── mti.xbus
        ├── lidar.pcap                 # raw Ouster UDP packets (all frames)
        ├── lidar_0.json                # ouster sensor metadata (companion to pcap)
        ├── radar.bag                  # ROS1 bag, points only, all frames
        └── gnss.nmea.log               # raw NMEA text, all sentences
```

Filenames are plain (`recording.svo2`, not `recording_<session_id>.svo2`) —
the session's own folder is what makes them unique, so a dataset root can
hold as many sessions side by side as you like and none of them can ever
collide with, or overwrite, another session's files.
`dataset.reader.list_sessions()` lists every session under a root, most
recently modified first; `dataset.reader.resolve_lidar_paths()` finds the
right pcap/JSON pair for a given (or the most recent) session. A session's
exact raw paths are also in its own
`<session_id>/metadata/session_manifest_<session_id>.json`.

## HDF5 Sensor File Schema

Each HDF5 file contains two groups:

### `/mti` — MTi-8 data (primary nav sensor)
| Dataset | dtype | Shape | Notes |
|---------|-------|-------|-------|
| `timestamp_ns` | int64 | (N,) | UTC epoch ns (converted from host monotonic via a mono->wall offset — see SYNCHRONIZATION.md), authoritative sync key |
| `host_timestamp_ns` | int64 | (N,) | same UTC epoch value as timestamp_ns |
| `device_timestamp` | int64 | (N,) | MTi's own free-running clock (`sample_time_fine \| sample_time_coarse<<32`), arbitrary epoch, not synced to host |
| `packet_counter` | int64 | (N,) | sequential, may gap on overflow |
| `acceleration` | float32 | (N,3) | m/s², body frame |
| `angular_velocity` | float32 | (N,3) | rad/s, body frame |
| `magnetometer` | float32 | (N,3) | uT, body frame |
| `quaternion_raw` | float32 | (N,4) | wxyz, raw sensor quaternion |
| `quaternion_canonical` | float32 | (N,4) | wxyz, gravity-aligned |
| `latitude` | float64 | (N,) | degrees |
| `longitude` | float64 | (N,) | degrees |
| `altitude_msl` | float64 | (N,) | meters |
| `altitude_ellipsoid` | float64 | (N,) | meters |
| `velocity` | float32 | (N,3) | m/s, NED |
| `temperature` | float32 | (N,) | Celsius |
| `baro_pressure` | float64 | (N,) | hPa |
| `status_word` | uint32 | (N,) | MTi status word |
| `status_byte` | uint8 | (N,) | MTi status byte |
| `delta_velocity` | float32 | (N,3) | preintegrated |
| `delta_quaternion` | float32 | (N,4) | preintegrated |

### `/zed_imu` — ZED camera IMU (diagnostic only, never for navigation)
| Dataset | dtype | Shape |
|---------|-------|-------|
| `timestamp_ns` | int64 | (N,) |
| `accel_x/y/z` | float32 | (N,) |
| `gyro_x/y/z` | float32 | (N,) |
| `quat_x/y/z/w` | float32 | (N,) |
| `temperature` | float32 | (N,) |

### File-level attributes
- `schema_version`: "2.0"
- `sensor_model`: "Xsens MTi-8"
- `compression`: "lzf"
- `coordinate_system`: "NED"
- `quaternion_raw_order`: "wxyz"

## Parquet Frame Index Schema

Each row = one **camera** frame, written unconditionally whenever recording
is on — never gated on a match with any other sensor. The `radar_*` columns
below are vestigial (kept for schema stability) and always empty/zero now
that radar has its own `metadata/radar/` index; `nearest_mti_timestamp_ns` /
`sync_delta_ns` are filled only when the live dashboard preview
(`Synchronizer.sync()`) happened to match at write time — `-1`/`0` otherwise,
never used to decide whether the row exists.

| Column | dtype | Notes |
|--------|-------|-------|
| `frame_id` | int64 | sequential from 0 |
| `timestamp_ns` | int64 | UTC epoch ns (`wall_timestamp_ns` from the camera grab), authoritative |
| `datetime_utc` | string | ISO format |
| `session_id` | string | recording session |
| `left_path` | string | relative: `left/prefix.png` (SVO2-only mode: always empty) |
| `right_path` | string | relative: `right/prefix.png` (SVO2-only mode: always empty) |
| `depth_path` | string | relative: `depth/prefix.npy` |
| `disparity_path` | string | relative: `disparity/prefix.npy` |
| `nearest_mti_timestamp_ns` | int64 | live-preview match only, converted to the same UTC-epoch basis as `timestamp_ns`; `0` if none |
| `sync_delta_ns` | int64 | live-preview match only; `-1` if none |
| `sensor_hdf5_file` | string | unused (always empty) |
| `nearest_mti_index` | int64 | unused (always `-1`) |
| `exposure` | float32 | camera exposure (us) |
| `gain` | float32 | camera gain |
| `temperature` | float32 | camera temperature |
| `radar_*` (8 columns) | — | vestigial, always empty/zero — see `metadata/radar/` |

## Parquet Radar Index Schema (`metadata/radar/`)

One row per **native** radar frame, independent of camera timing.

| Column | dtype | Notes |
|--------|-------|-------|
| `frame_id` | int64 | AWR1843 frame number |
| `timestamp_ns` | int64 | host monotonic, at frame decode |
| `points_path` | string | relative JSON point-cloud path, empty when no points |
| `heatmap_path` | string | relative uint16 range–Doppler NPY path |
| `range_profile_path` | string | relative uint16 range-profile NPY path, when enabled |
| `num_objects` | int64 | detected-point count reported by the radar |
| `num_range_bins` / `num_doppler_bins` | int32 | heatmap dimensions |

## Parquet GNSS Index Schema (`metadata/gnss/`)

One row per parsed GGA/RMC fix, independent of camera timing. Columns:
`timestamp_ns`, `latitude`, `longitude`, `altitude`, `fix_quality`,
`fix_type`, `satellites`, `hdop`, `speed_mps`, `course_deg`, `rtk_fixed`,
`utc_time`, `gps_week`, `gps_tow`. Never includes NTRIP credentials.

## Parquet Synchronization Index Schema (`metadata/synchronization/`)

**Derived, offline only** — built by `synchronize-dataset` /
`fusion.offline_sync.synchronize_session()` after recording, never during.
One row per camera frame: `camera_timestamp_ns`, `camera_frame_id`, and for
each of imu/radar/lidar/gnss: `{modality}_timestamp_ns`,
`dt_camera_{modality}_ns` (residual, `-1` if no sample exists in that
modality), `{modality}_in_tolerance` (bool). See `SYNCHRONIZATION.md`.

## Radar Records

Radar points/heatmap/range-profile are written for **every native radar
frame**, independent of camera timing (see `metadata/radar/` above and
`DATASET_WRITER.md`). A point-cloud JSON contains the frame's own timestamp,
frame ID, reported object count, and every point's `x`, `y`, `z` (metres) and
`velocity_mps`. The range–Doppler heatmap and range profile are unsigned
magnitude values emitted by the TI SDK; they are not calibrated dB
measurements. **Raw ADC/IQ samples are not captured** — only the TI mmWave
SDK's post-processed TLV output is available from this hardware interface.

## Lidar Records

The Ouster OS-0-64 (64 channels, ~10 Hz) produces **exactly one pcap and one
companion config JSON per recording session** — never one JSON per
frame/scan/packet. This is the sole lidar recording; there is no second,
per-frame lidar output.

### Raw packet capture (PCAP + config JSON)

`<session_id>/raw/lidar.pcap` holds the raw Ouster UDP packets for **every** lidar frame
received during the session, regardless of camera/IMU synchronization — this
is the primary raw Ouster dataset. A pcap alone is not self-describing, so
the sensor configuration/metadata is written once, at session start, to
`<session_id>/raw/lidar_0.json` (the `<stem>_0.json` companion that `ouster-cli` / pcap
readers, and the SDK's own `open_source()` auto-discovery, expect). That
JSON is session-level configuration only (beam angles, calibration, lidar
mode) — it carries no per-frame data.

### Deriving scans offline

There is no pre-generated per-frame JSON to read. Individual scans/point
clouds are decoded on demand from the pcap + config JSON by
`dataset.reader.iter_lidar_frames(pcap_path, meta_path)`, using the Ouster
SDK's `open_source()`. Each yielded scan:

| Field | Type | Notes |
|-------|------|-------|
| `frame_id` | int64 | Ouster scan/frame number |
| `timestamp_ns` | int64 | from the pcap's own recorded packet-capture time (`frame.get_first_valid_packet_timestamp()`) — not fabricated, not read from a side file |
| `points` | `(N,3)` float32 | XYZ cloud in metres, valid returns only |
| `reflectivity` | `(N,)` float32 or `None` | per-point reflectivity when the field is present |

`DatasetReader.lidar_frames()` / `get_lidar_frame(frame_id)` wrap this
generator with in-memory caching for callers that want repeated access; the
underlying decode always starts from the pcap, never a cache file. For
replay in third-party tools, the raw `lidar.pcap` + `lidar_0.json` pair is
the canonical, directly-usable source. Camera-frame alignment with a
specific lidar scan is available only via the derived
`metadata/synchronization/` index, built offline
(`synchronize-dataset`/`fusion/offline_sync.py`).

## Image Naming Convention

```
YYYYMMDD_HHMMSS_NNNNNNNNN_FFFFFFFFFF.ext
│         │     │         │
date      time  nano_frac  frame_id
```

- `date/time`: human-readable UTC (approximate)
- `nano_frac`: nanosecond fraction of timestamp_ns
- `frame_id`: from `ZEDCamera.frame_id` (resets on camera reopen)
- **Filenames are for human readability only** — never derive timestamps from them

## Session Metadata

| Column | dtype | Notes |
|--------|-------|-------|
| `session_id` | string | unique per recording |
| `start_ns` | int64 | monotonic_ns at REC |
| `end_ns` | int64 | monotonic_ns at STOP |
| `status` | string | ACTIVE or COMPLETED |
| `frames_start` | int64 | frame_count at begin |
| `frames_end` | int64 | frame_count at end |
| `mti_start_index` | int64 | MTi sample index at begin |
| `mti_end_index` | int64 | MTi sample index at end |

## Key Design Decisions

1. **Never silently drop samples** — HDF5 queue overflow is tracked and reported
2. **Never overwrite files** — filename collision appends `_001`, `_002` suffix
3. **Never reduce float precision** — float32 for sensor data, float64 for GPS
4. **Never derive timestamps from filenames** — filenames are approximations
5. **Never allow compression in camera thread** — PNG write is async
6. **Never let one failed session invalidate others** — each session is independent
7. **Never delete legacy datasets** — v1 datasets coexist alongside v2
