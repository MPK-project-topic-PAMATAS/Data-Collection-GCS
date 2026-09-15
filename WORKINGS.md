# Workings — Ground Control Station

How the GCS is wired up: the hardware connections, the threads and queues
that move data around, and exactly what happens when you press REC.

Companion docs: `SYNCHRONIZATION.md` (time-matching in depth),
`docs/DATASET_FORMAT.md` (on-disk schema).

## Launch

Two entry points, one pipeline core:

| Command | What runs | Use when |
|---------|-----------|----------|
| `record` / `gcs` (→ `run_gcs.py`) | PySide6 GCS + live sensors | Live capture with monitoring |
| `gcs --demo` | `main_window.run_demo()` | No hardware, simulated frames |
| `python main.py` | Headless CPU loop | Batch / headless recording |

Both live paths instantiate the same objects:

```
ZEDCamera  ─┐
MTiReceiver─┤→ Synchronizer ─→ AcquisitionThread ─→ DatasetWriter
GNSSReceiver┘                                      (async queue)
                     └── publishes SharedFrame ──→ Qt dashboard
```

The GCS additionally wires Recording UI signals (`recorder_panel.sig_start/stop`)
to start/stop hooks; `main.py` calls begin/end session directly.

## Hardware connections

| Device | Interface | Config | Role |
|--------|-----------|--------|------|
| ZED 2 stereo camera | USB 3.x | HD720 @ 30 FPS, depth NEURAL_LIGHT | Drives the pipeline; stereo + depth + onboard IMU |
| Xsens MTi-8 | Serial, auto-detected (ttyUSB/ttyACM/name) | 2,000,000 baud, Xbus, MTData2 | Sole navigation/orientation sensor @ ~200 Hz |
| u-blox F9P GNSS | Serial `/dev/ttyUSB1`, 115200 | NMEA GGA/RMC + NTRIP RTK (VRS) | Geotagging, RTK fix, yaw drift correction |

- MTi port/baud: `config/settings.py` (`MTI_PORT`, `MTI_BAUDRATE`). The reader
  auto-detects the port if the configured one is absent.
- GNSS is optional; `run_gcs.py --no-gnss` skips it. NTRIP credentials come
  from `config/ntrip_credentials.py`.

## Components (inner workings)

### ZEDCamera (`camera/zed_camera.py`)

Open → read intrinsics/extrinsics/baseline → then per grab:
`grab()` → both rectified images, raw depth (+ optional
disparity/confidence), dashboard views, onboard IMU (accel, gyro, quaternion
`(x,y,z,w)`, magnetometer, temperature), exposure/gain/temperature metrics.
Frames are stamped with `time.monotonic_ns()` (matching) and
`time.time_ns()` (wall, for persistence). `CAMERA_FLIP_180` rotates images if
the camera is mounted upside-down. `start_recording()` enables the SVO2
codec stream (`recording_<session_id>.svo2`).

The ZED IMU is **diagnostic only** (`ENABLE_ZED_IMU_LOGGING`, default off).
MTi-8 is the navigation source — never load calibration alignment for the ZED
IMU (alignment failed in the past, 40° RMS residual).

### MTiReceiver (`mti8/receiver.py`)

Daemon thread: `SerialReader` (raw serial bytes, optional raw log) →
`XbusParser` (packet framing) → MTData2 decode → monotonic stamp →
`MTiValidator` (NaN/Inf rejection, quaternion norm, packet-counter gaps,
rotation-discontinuity) → ring buffer `deque(maxlen=300)` ≈ 1.5 s @ 200 Hz.

`get_nearest(target_ts)` scans the buffer newest-first and returns the first
local time minimum — zero contention per frame.

### GNSSReceiver (`gnss/receiver.py`)

Daemon thread reads NMEA over serial, parses GGA (position/fix/HDOP) and RMC
(UTC time, speed, course). GGA is echoed to the NTRIP caster for RTK
corrections in the same thread. Keeps only the **latest** fix dict
(no history, timestamped). RMC UTC is converted to GPS week/TOW.

### RadarReceiver (`radar/receiver.py`)

Daemon thread over two serial ports (CFG + DATA). Decodes TI mmWave TLV
frames — detected points (TLV1) and range-Doppler heatmap (TLV5); range
profile (TLV2) is parsed but not enabled in the sensor config by default.
**No raw ADC/IQ** — that would need LVDS/DCA1000 capture hardware, not
present. Ring buffer `deque(maxlen=RADAR_BUFFER_SIZE)`; `get_new_since()`
drains every frame since the last poll for native-rate recording. Tracks
dropped frames from the sensor's own `frame_number` header sequence.

### LidarReceiver (`lidar/receiver.py`)

Daemon thread wrapping the Ouster SDK's live source. Every incoming frame's
raw packets are appended to a single `lidar.pcap` **in this same ingest
thread**, independent of anything else in the pipeline. Exactly one
companion config JSON (`lidar_0.json`, sensor beam angles/calibration/mode)
is written once, when recording starts — never one JSON per frame. Individual
scans are decoded from the pcap on demand, offline, by
`dataset.reader.iter_lidar_frames()`, keyed by the packet-capture timestamps
already inside the pcap itself. Tracks dropped frames from the sensor's own
`frame_id` sequence.

### Synchronizer (`fusion/synchronizer.py`)

**Live preview only** — does not gate recording. Per camera grab: nearest
IMU sample within the 20 ms gate (`MAX_DELTA_NS`), nearest radar/lidar within
their own gates, latest GNSS fix → `SyncedFrame`, used solely for dashboard
display and two informational Parquet columns. Full detail in
`docs/SYNCHRONIZATION.md`.

### AcquisitionThread (`dashboard/acquisition.py`)

The camera hot path only — MTi/radar/GNSS structured recording happens in
separate poller threads (below), not here. Each camera grab:
1. BGR→RGB for display
2. `sync.sync(frame)` — for dashboard preview only, result never gates writing
3. Pick up latest MTi accel/gyro/quat (last-valid held across gaps, display only)
4. Pick up ZED accel/gyro/quat (dashboard visibility only)
5. If recording: `writer.write_camera_frame(frame, synced)` (enqueue,
   non-blocking, **always** — `synced` being `None` never skips this)
6. Publish a fresh `SharedFrame` snapshot (lock-free latest-value buffer)

### Native-rate structured pollers (`dataset/*_structured_recorder.py`)

`MTiStructuredPoller`, `RadarStructuredPoller`, `GNSSStructuredPoller` —
started/stopped alongside the raw recorders in `run_gcs.py`, each its own
daemon thread. They drain their receiver's buffer (`get_new_since()`) and
call `writer.write_mti_sample()` / `write_radar_frame()` / `write_gnss_fix()`
independently of the camera loop — a camera stall never stops any of these.

### DatasetWriter (`dataset/writer.py`)

Async writer thread + bounded queue (`maxsize=128`), now dispatching four
independent entry points (camera/mti/radar/gnss), each enqueued by its own
caller — no cross-sensor gating:

- **Camera** — depth/disparity NPY (optional confidence), frame index row
  (always written, whether or not any other sensor matched).
- **MTi → HDF5** — every native packet, `host_timestamp_ns` +
  `device_timestamp` (the MTi's own clock) + `packet_counter` preserved.
- **Radar → JSON/NPY + Parquet index** — every native frame's points,
  heatmap, range profile.
- **GNSS → Parquet index** — every parsed fix.
- Lidar has no writer method — `lidar/receiver.py` already persists it.

### HDF5SensorWriter (`dataset/hdf5_sensor_writer.py`)

Own thread, owns the `h5py.File` (never shared across threads). Samples arrive
through per-sensor bounded queues and are drained in **batches** (matching the
HDF5 chunk layout, max 256 samples or 50 ms). Rotates files on UTC date change
(`YYYYMMDD_000000_mti.h5`). File attrs record schema, sensor model, baud,
coordinate system (NED), quaternion order (`wxyz` — XSens wire), convention.

### Parquet metadata writers

- `FrameIndexWriter` — `metadata/frames/date=YYYYMMDD/part-NNNNNN.parquet`,
  flushed every 1000 rows or on `flush()`.
- `SessionWriter` — one row per REC→STOP cycle in `metadata/sessions/`.
- `EventWriter` — RECORDING_START/STOP (and any marked events) in
  `metadata/events/`.

## How a recording happens

1. Press **REC** in the dashboard.
2. `run_gcs.py::on_start()` runs the **health gate** (`dashboard/gate.py`):
   camera open (critical), MTi running (critical), ≥ 1.5 GB disk free
   (critical), calibration files present (critical), at least one recording
   output enabled (critical); GNSS/radar/lidar/sync are informational only —
   none of them blocks recording of the sensors that *are* healthy.
3. On pass:
   - `writer.begin_session(ns)` — opens session row + RECORDING_START event
   - a config snapshot is saved (`metadata/config_snapshot_<session_id>.json`)
   - `acq.set_recording(True)` — camera frames start enqueueing
   - `camera.start_recording(recording_<session_id>.svo2)` — SVO2 stream recorded
   - `mti.start_raw_log(mti_<session_id>.xbus)` — raw Xbus bytes teed to disk, verbatim
   - `gnss.start_raw_log(gnss.nmea.log)` — raw NMEA teed to disk, verbatim
   - lidar pcap, radar bag, and the MTi/radar/GNSS structured pollers all
     start — each independent of the others and of the camera
4. While recording, every sensor writes at its own rate — see "Native-rate
   structured pollers" above.
5. Press **STOP** → `on_stop()`:
   - all pollers/recorders stopped, drop/gap counters recorded as events
   - `lidar.stop_pcap()` finalizes the session's single `lidar.pcap` (its
     one companion `lidar_0.json` was already written when recording started)
   - `writer.end_session(ns)` — writes session row (COMPLETED) + RECORDING_STOP
     event, then `_flush_all()` (Parquet + HDF5)
   - a per-session manifest is written
     (`metadata/session_manifest_<session_id>.json`, see
     `dataset/session_manifest.py`) listing exactly which files this
     session produced (SVO2, xbus, the one pcap + one config JSON, bag,
     NMEA log), start/stop times, and synchronization tolerances
6. On app exit, `writer.close()` drains the queue, stops HDF5, writes
   `manifests/dataset_manifest.json` (schema, device versions, counts, drops,
   file list, calibration SHA) — this is the cumulative, dataset-wide
   summary, distinct from the per-session manifest above.
7. **Offline, separately**: `synchronize-dataset output/` builds
   `metadata/synchronization/` — nearest-match + residuals across all
   recorded streams, decoding lidar scans from the pcap on the fly. This
   never runs during recording.

### Dataset layout (root = `output/`)

```
output/
├── depth/ disparity/ confidence/                 # per-camera-frame files
├── sensors/YYYYMMDD_000000_mti.h5                # MTi, native ~200 Hz
├── radar/points|heatmap|range_profile/            # native per-radar-frame
├── metadata/
│   ├── frames|radar|gnss/date=YYYYMMDD/            # native-rate Parquet indexes
│   ├── synchronization/date=YYYYMMDD/              # DERIVED, offline only
│   ├── session_manifest_<session_id>.json          # per-session file list
│   ├── sessions|events/date=YYYYMMDD/
│   └── config_snapshot_<session_id>.json
├── calibration/  zed_intrinsics.yaml, camera_info.json, camera_imu.yaml,
│                 extrinsics/*.yaml
├── manifests/dataset_manifest.json
└── raw/recording.svo2, mti.xbus, lidar.pcap, lidar_0.json, radar.bag,
    gnss.nmea.log
```

## Dashboard (PySide6, Qt main thread)

Renders only the latest `SharedFrame` snapshot; never touches acquisition.

Tier 1 (top): stereo + depth/confidence feeds (with depth-view toggle).
Tier 2 (middle): IMU panel + 3D viewport.
Tier 3 (bottom): system health, quality, recording, event log.

Key panels:
- **IMU panel** — MTi primary + ZED diagnostic side by side, roll/pitch/yaw
  deltas and quaternion angular error. MTi uses `_mti_quat_to_euler` (XSens
  `w,x,y,z`); ZED uses `_zed_quat_to_euler` (ZED `x,y,z,w`) — the two differ.
- **Heading** — `GNSSYawCorrector` pulls displayed MTi yaw toward GNSS course
  when moving (>1.5 m/s). Display-only; raw MTi is recorded unmodified.
- **Map panel** — OSM tiles at zoom 15, current position, accuracy circle
  (radius = `hdop × HDOP_TO_METERS`), green trail (cap 1,000 pts).
- **Sync panel** — live residual, percentiles, rejected %, histogram with
  warn/reject reference lines at 15/20 ms.
- **Recorder / System / Quality** — session state, connection health, frame
  & IMU rates, disk free.

Quaternion formats are not interchangeable: MTi-8 is `(w,x,y,z)` (XSens wire),
ZED SDK is `(x,y,z,w)` (RIGHT_HANDED_Z_UP). Any new orientation code must pick
per sensor. See `mti8/frame.py` for the authoritative statement.

## Thread model

| Thread | Owns | Writes |
|--------|------|--------|
| MTi receiver | serial → parse → validate | ring buffer |
| Radar receiver | serial → TLV parse | ring buffer |
| Lidar receiver | Ouster SDK ingest | ring buffer, `lidar.pcap` (+ one `lidar_0.json` config JSON, always) |
| GNSS receiver | serial NMEA + NTRIP | latest fix + fix buffer |
| AcquisitionThread | grab → color → sync (preview) → snapshot | writer queue (camera only), SharedFrame |
| MTi/Radar/GNSS structured pollers | drain buffer at native rate | writer queue (mti/radar/gnss) |
| RadarBagPoller | poll radar at native rate | `radar.bag` |
| Dataset writer | NPY/HDF5/JSON/Parquet/metadata I/O | filesystem |
| HDF5 writer | h5py file exclusively | `sensors/*.h5` |
| Qt main | rendering | — (reads snapshots) |

## Key files

| File | Responsibility |
|------|----------------|
| `run_gcs.py` | Live wiring: sensors, sync, writer, recording hooks, pollers |
| `dashboard/acquisition.py` | `AcquisitionThread`, `SharedFrame` (camera path only) |
| `dashboard/main_window.py` | Window, tiers, timers, euler converters |
| `fusion/synchronizer.py` | Live preview matching (display only, never gates recording) |
| `fusion/offline_sync.py` | Derived, offline nearest-match + residuals |
| `dataset/writer.py` | Async dataset writer — per-sensor independent write methods |
| `dataset/mti_structured_recorder.py`, `radar_structured_recorder.py`, `gnss_structured_recorder.py` | Native-rate pollers |
| `dataset/config_snapshot.py` | Immutable per-session config snapshot |
| `dataset/hdf5_sensor_writer.py` | HDF5 IMU storage, rotation |
| `dataset/parquet_index.py`, `session.py`, `events.py` | Metadata indexes (frames/radar/gnss/synchronization/sessions/events) |
| `mti8/receiver.py`, `validator.py` | MTi ingestion & validation |
| `radar/receiver.py` | AWR1843 TLV ingestion |
| `lidar/receiver.py` | Ouster ingestion + native pcap/JSON recording |
| `gnss/receiver.py`, `ntrip_client.py` | GNSS + RTK |
| `config/settings.py` | All tunables (fps, depth, flip, ports, gates) |