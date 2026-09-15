# Multi-Sensor Dataset Recording

## Overview

Every sensor is recorded independently at its own native rate — a missing
camera frame never removes an IMU/radar/lidar/GNSS sample. Synchronization
across sensors is a separate, derived, offline step
(`synchronize-dataset`/`fusion/offline_sync.py`) — see `SYNCHRONIZATION.md`.

- **Camera**: SVO2 (video + depth), plus a per-frame Parquet index
- **IMU**: raw Xbus log + HDF5 (structured, at MTi's own ~200 Hz)
- **Radar**: raw ROS1 BAG (points only) + native JSON/heatmap/NPY + Parquet index
- **LiDAR**: **exactly one PCAP + one config JSON per session** (never
  per-frame/scan/packet files), written directly by `lidar/receiver.py`,
  always independent of the camera — scans are decoded from the pcap
  offline, on demand
- **GNSS**: raw NMEA log + Parquet fix index, at native GNSS rate
- **Session manifest**: one JSON per session listing every file path above
  plus start/stop timing and sync metadata (`metadata/session_manifest_<id>.json`)

## Recording Settings

Located in `config/settings.py`:

```python
MAX_FRAMES = 1800          # 60 seconds @ 30 FPS
SAVE_SVO = True            # Camera video + depth
SAVE_RAW_IMU = True        # MTi-8 data in HDF5
SAVE_RADAR_BAG = True      # Radar in ROS bag
# LiDAR pcap + one config JSON enabled automatically whenever lidar is present
```

## Storage Usage

| Format | Size/min | Duration @ 5.9 GB |
|--------|----------|-------------------|
| LiDAR PCAP | 30-50 MB | 80-120 min |
| Camera SVO | 200 MB | 25-30 min |
| IMU HDF5 | 5 MB | 1000+ min |
| Radar BAG | 50 MB | 100-150 min |
| **Total** | **~300 MB/min** | **~20 min** |

The lidar config JSON is a few KB, written once per session — negligible.

## Directory Structure

```
output/
├── sensors/
│   └── 20260907_171900_mti.h5   (IMU time series)
├── radar/
│   ├── points/*.json            (detected objects, native rate)
│   ├── heatmap/*.npy            (range-Doppler, native rate)
│   └── range_profile/*.npy      (range signature, when enabled)
├── raw/
│   ├── lidar.pcap                 (the ONE Ouster pcap for the session)
│   └── lidar_0.json               (the ONE config JSON, companion to it)
├── metadata/
│   ├── frames/*.parquet         (one row per camera frame, always written)
│   ├── radar/*.parquet          (one row per native radar frame)
│   ├── gnss/*.parquet           (one row per native GNSS fix)
│   ├── synchronization/*.parquet (DERIVED — built offline, see SYNCHRONIZATION.md)
│   ├── sessions/*.parquet       (session metadata)
│   ├── events/*.parquet         (recording events, incl. drop/gap counts)
│   ├── config_snapshot_<session_id>.json  (immutable, no secrets)
│   └── session_manifest_<session_id>.json  (file paths + start/stop + sync tolerances)
└── calibration/
    ├── zed_intrinsics.yaml
    ├── camera_imu.yaml
    └── extrinsics/
```

## Recording Performance

### Writing Architecture

All I/O is **asynchronous** with bounded queues:
- Camera frame: enqueued via `writer.write_camera_frame()` (non-blocking)
- MTi/radar/GNSS samples: enqueued via their own structured poller threads
  (`dataset/mti_structured_recorder.py` etc.), independent of the camera loop
- Writer thread: dispatches by sensor tag to async SVO, HDF5, Parquet, JSON writes
- Max queue size: 128 items (shared across all four sensor write paths)
- Dropped items logged if queue full (`writer.dropped`)

### CPU/GPU Usage

- **GPU**: ZED depth inference stays under 20% (NEURAL_LIGHT mode)
- **CPU**: Writing thread separate, minimal main acquisition impact
- **Memory**: ~500 MB steady-state
- **Real-time**: Maintains 30 FPS camera acquisition

## Reading Datasets

### Quick Start

```python
from dataset.reader import DatasetReader

# Load dataset — lidar scans are decoded lazily from <session_id>/raw/lidar.pcap +
# <session_id>/raw/lidar_0.json on first access; there is no per-frame file to read.
reader = DatasetReader("output")

for frame in reader.lidar_frames():
    print(f"Frame {frame.frame_id}: {len(frame.points)} points")
    print(f"  XYZ range: {frame.points.min(axis=0)} to {frame.points.max(axis=0)}")
```

### Inspect Dataset

```bash
python playback_dataset.py --dataset output
```

Output:
```
📍 LiDAR:
   • PCAP: lidar.pcap (42.1 MB)
   • Config JSON present: True
   • First scan frame_id: 4282
   • Points in first scan: 65536
   • X range: [-8.50, 8.50] m
   • Y range: [0.10, 25.00] m
   • Z range: [-1.50, 2.50] m

📹 Camera:
   • SVO2 files: 1
   • output.svo2: 210.5 MB

📊 IMU:
   • HDF5 files: 1
   • 20260907_171900_mti.h5: 4.2 MB

📡 Radar:
   • ROS bag files: 1
   • output.bag: 52.3 MB

📋 Metadata:
   • Frame index: 1 parquet files
   • Sessions: 1
   • Events: 3
```

## Format Details

### LiDAR (one PCAP + one config JSON — never per-frame files)

**PCAP** (`<session_id>/raw/lidar.pcap`):
- Raw UDP packets from Ouster sensor, for the whole session
- Playable in Ouster Studio
- Requires `<session_id>/raw/lidar_0.json` (config metadata) in the same directory
- Size: ~30-50 MB/min (depends on frame rate)
- This is the only lidar recording — there is no second, per-frame output

**Config JSON** (`<session_id>/raw/lidar_0.json`):
- Sensor configuration (beam angles, calibration, lidar mode) — session-level
  metadata only, not per-frame data
- Written once, at recording start, by `LidarReceiver.start_pcap()`
- Required for PCAP replay in Ouster Studio and for
  `dataset.reader.iter_lidar_frames()`

**Deriving scans** (no files on disk — computed on demand):
- `dataset.reader.iter_lidar_frames(pcap_path, meta_path)` decodes
  individual scans from the pcap + config JSON using the Ouster SDK
- Each scan's timestamp comes from the pcap's own packet-capture time
  (`frame.get_first_valid_packet_timestamp()`), not a side file
- `DatasetReader.lidar_frames()` / `get_lidar_frame(frame_id)` wrap this with
  in-memory caching

### Camera (SVO2)

**Format**: Stereolabs ZED SDK native format
- Contains: stereo video + depth + IMU + metadata
- Playable via: `python -c "from pyzed import sl; ..."`
- Size: ~200 MB/min at HD720 30 FPS

### IMU (HDF5)

Samples are appended at the MTi's own native rate (~200 Hz), independent of
the camera — not downsampled to camera FPS.

**Layout** (`sensors/YYYYMMDD_HHMMSS_mti.h5`, group `/mti`), with distinct
timestamp fields per `SYNCHRONIZATION.md`'s clock-domain split:

```
/mti/
  ├── timestamp_ns        (int64, UTC epoch ns — converted from host CLOCK_MONOTONIC
  │                                via a mono->wall offset at write time; same as host_timestamp_ns)
  ├── host_timestamp_ns   (int64, same UTC epoch value as timestamp_ns)
  ├── device_timestamp    (int64, MTi's own free-running clock: sample_time_fine | sample_time_coarse<<32,
  │                                arbitrary epoch, NOT synced to host)
  ├── packet_counter      (int64, MTi's own sequence number — used for drop detection)
  ├── acceleration        (Nx3 float32, m/s²)
  ├── angular_velocity    (Nx3 float32, rad/s)
  ├── magnetometer        (Nx3 float32, normalized)
  ├── quaternion_raw      (Nx4 float32, w,x,y,z — XSens wire order)
  └── ... (velocity, temperature, baro_pressure, status_word/byte, delta_velocity, delta_quaternion)
```

See `DATASET_FORMAT.md` for the full schema.

**Loading**:
```python
import h5py
with h5py.File("sensors/20260907_171900_mti.h5") as f:
    timestamps = f["mti"]["timestamp_ns"][:]
    accel = f["mti"]["acceleration"][:]
    packet_counter = f["mti"]["packet_counter"][:]
```

### Radar (ROS1 Bag + native JSON/NPY)

The raw `.bag` (`<session_id>/raw/radar.bag`) carries **points only**, at native radar
rate, written by `dataset/radar_bag_recorder.py`:
- Topic: `/radar/points` (`sensor_msgs/PointCloud2`, fields `x,y,z,velocity_mps`)
- Topic: `/clock` (`rosgraph_msgs/Clock`)

The range-Doppler heatmap and range profile are **not** in the bag — they are
written as `radar/heatmap/*.npy` / `radar/range_profile/*.npy` by
`RadarStructuredPoller`, at the same native rate, indexed in
`metadata/radar/*.parquet`.

**Reading the bag** (this is a ROS1, not ROS2, bag — use `rosbags.rosbag1`):
```python
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
ts = get_typestore(Stores.ROS1_NOETIC)
with Reader("<session_id>/raw/radar.bag") as reader:
    for conn, timestamp, rawdata in reader.messages():
        print(f"{conn.topic}: {len(rawdata)} bytes")
```

**Raw ADC/IQ is not captured** — this driver only consumes the TI mmWave
SDK's post-processed TLV UART output, not LVDS/DCA1000 raw capture.

### Metadata (Parquet)

**Frames** (`metadata/frames/*.parquet`):
- One row per camera frame, always written (never gated on another sensor)
- Columns: frame_id, timestamp_ns, depth_path, exposure/gain/temperature, ...
- Query via pandas: `pd.read_parquet("output/metadata/frames/*.parquet")`

**Radar** (`metadata/radar/*.parquet`) / **GNSS** (`metadata/gnss/*.parquet`):
- One row per native radar frame / GNSS fix, independent of camera timing

**Synchronization** (`metadata/synchronization/*.parquet`) — **derived**:
- Built offline by `synchronize-dataset`; nearest-match + residuals across
  all recorded streams, never written during recording

## Performance Tips

### Recording Long Sessions

If available space is limited:

1. **Disable non-essential modalities**:
   ```python
   SAVE_SVO = False          # Save 200 MB/min
   SAVE_RADAR_BAG = False    # Save 50 MB/min
   ```

2. **Reduce frame count**:
   ```python
   MAX_FRAMES = 1200         # 40 seconds instead of 60
   ```

3. **Move output directory**:
   ```python
   DATASET_DIR = "/mnt/external/dataset"
   ```

### Loading Large Datasets

**Streaming decode** (for long sessions — don't load every scan into memory
at once): `iter_lidar_frames` is a generator, so iterate it directly instead
of going through `DatasetReader.lidar_frames()` (which caches everything):
```python
from dataset.reader import iter_lidar_frames

for frame in iter_lidar_frames("output/<session_id>/raw/lidar.pcap", "output/<session_id>/raw/lidar_0.json"):
    process(frame)  # process and discard, one scan at a time
```

## Troubleshooting

### PCAP Not Openable in Ouster Studio

**Error**: "We were unable to parse this config"

**Solution**: Ensure the config JSON is in the same directory as the pcap —
there is exactly one of each per session:
- PCAP: `output/<session_id>/raw/lidar.pcap`
- Config JSON: `output/<session_id>/raw/lidar_0.json`

### SVO2 File Too Large

**Solution**: Disable depth saving (already disabled by default):
```python
SAVE_DEPTH = False
```

### Recording Drops Frames

Check dashboard status:
- If "Dropped: 5" → writer queue full
- Solution: Enable async I/O (already enabled)

## Development Notes

### Recording Thread Architecture

Each sensor's recording path is independent — none of them wait on, or are
gated by, another:

```
AcquisitionThread          MTiStructuredPoller   RadarStructuredPoller   GNSSStructuredPoller
├─ Camera grab (30 Hz)     ├─ drain MTi buffer    ├─ drain radar buffer   ├─ drain GNSS buffer
├─ sync() for preview      │  (~200 Hz)           │  (~6.7 Hz)            │  (~1-10 Hz)
└─ write_camera_frame()    └─ write_mti_sample()  └─ write_radar_frame()  └─ write_gnss_fix()
   (always, unconditional)

LidarReceiver's own ingest thread — appends every frame's packets to the
single session pcap directly, always, independent of all of the above.
Scans are never written per-frame; they're decoded from that pcap offline.

                    all four ──────▶  DatasetWriter's single bounded queue
                                              │
                                       Writer Thread
                                       ├─ Write SVO chunk (camera, via SDK)
                                       ├─ Write HDF5 sample (mti)
                                       ├─ Write JSON/NPY + Parquet row (radar)
                                       ├─ Write Parquet row (camera, gnss)
                                       └─ never blocks any producer thread

(offline, after recording) synchronize-dataset → metadata/synchronization/
```

### Adding New Sensors

1. Define a `SensorFrame` dataclass
2. Add its own independent `write_*` method to `DatasetWriter` — never make
   it depend on another sensor's data being present
3. If the receiver doesn't already buffer samples, add `get_new_since()`
   (see `mti8/receiver.py`/`radar/receiver.py`) and a structured poller
   (see `dataset/mti_structured_recorder.py`)
4. Add the new modality to `fusion/offline_sync.py`'s nearest-match loop
5. Add playback/inspection logic to `playback_dataset.py`

## See Also

- `dataset/writer.py` — Writing logic (per-sensor independent write methods)
- `dataset/mti_structured_recorder.py`, `radar_structured_recorder.py`,
  `gnss_structured_recorder.py` — native-rate structured pollers
- `dataset/reader.py` — Reading logic, incl. `iter_lidar_frames()` (pcap decode)
- `lidar/receiver.py` — Ouster ingest + the single pcap + config JSON recorder
- `dataset/session_manifest.py` — per-session file-path manifest
- `fusion/offline_sync.py`, `synchronize_dataset.py` — derived synchronization
- `playback_dataset.py` — Inspection tool
- `config/settings.py` — Recording configuration
- `SYNCHRONIZATION.md`, `DATASET_WRITER.md`, `DATASET_FORMAT.md` — architecture
