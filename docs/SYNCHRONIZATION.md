# Synchronization Model

Synchronization is a **derived, offline operation** — it never gates recording.
Every sensor is recorded independently at its own native rate; a missing camera
frame never removes an MTi/radar/lidar/GNSS sample, and vice versa. The source
of truth for live/preview matching is `fusion/synchronizer.py` and
`fusion/synced_frame.py`; the source of truth for the derived, post-recording
index is `fusion/offline_sync.py`.

## Clock model

All sensors run on the **host monotonic clock** (`time.monotonic_ns()`) for
every `timestamp_ns` field (camera, MTi, radar, lidar, GNSS). There is **no
PTP and no hardware PPS/trigger implemented anywhere in this codebase** —
matching that claim to reality is a hard requirement. Wall-clock (epoch) time
is derived only where needed (filenames, ROS bag timestamps) via a captured
`time.time_ns() - time.monotonic_ns()` offset, never used for matching itself.

MTi additionally carries a **device clock** timestamp (`sample_time_fine` /
`sample_time_coarse`, combined into `device_timestamp` in the HDF5 schema) —
this is the Xsens IMU's own free-running clock, with an arbitrary epoch from
device power-on and no rollover handling in this codebase. It is preserved
alongside `packet_counter` and the host `timestamp_ns` but is **not** used for
cross-sensor matching (there is no established mapping from the MTi's clock
to the host's), and it is **not** a PPS/GPS-disciplined value — it is
purely the device's own gyroscope-driven timer. GNSS carries a genuine
sensor-reported time (`utc_time`/`gps_week`/`gps_tow` parsed from `$GPRMC`),
which is likewise preserved but not used to discipline any other clock.

The architecture leaves room for a future hardware PPS input without a
redesign: it would arrive as one more explicit field (e.g.
`pps_timestamp_ns`) alongside the existing device-clock/host-clock split,
consumed the same way `device_timestamp` already is — documented, not
claimed as implemented today.

> Host `timestamp_ns` is authoritative for all matching. Never reconstruct
> wall time from filenames — filenames are readability aids only.

Per-source timestamp capture point:

| Source      | Stamp taken at                                        | Where                              |
|-------------|--------------------------------------------------------|-------------------------------------|
| ZED frame   | immediately after a successful SDK grab                | `camera/zed_camera.py` (`grab()`)  |
| MTi sample  | packet arrival time, after Xbus decode                 | `mti8/receiver.py` (`_loop()`)     |
| GNSS fix    | arrival time of last NMEA sentence (kept in fix dict)  | `gnss/receiver.py`                 |

There is **no hardware trigger or PTP** between devices — this is software
timestamping at arrival; the acceptance windows below absorb the resulting
USB/serial jitter.

## Optional Ouster PTP / PPS timestamping and offset estimation

The Ouster is the only sensor here that can take an external time source.
Set `config.settings.LIDAR_TIMESTAMP_MODE` to `"TIME_FROM_PTP_1588"` (run
`ptp4l` on the laptop NIC with the laptop as grandmaster, ideally with
`chrony` disciplined by the MTi/GNSS 1PPS) or `"TIME_FROM_SYNC_PULSE_IN"`.
`None` (default) leaves the sensor untouched. The mode is applied once at
`LidarReceiver.start()` (the sensor re-initialises only if it differs), the
active mode is recorded in the config snapshot (`lidar.timestamp_mode_active`),
and each live `LidarFrame` carries `sensor_timestamp_ns` (sensor clock).
Recorded `timestamp_ns` values are still host-arrival stamps; PTP makes the
sensor clock comparable to the host clock, it does not rewrite them.

The ZED 2 and AWR1843 have no sync input, so their latency is estimated
offline from a recorded calibration drive:

```bash
python calibration/estimate_time_offsets.py output/ --session <SID> --out offsets.json
```

It cross-correlates |gyro| (ZED IMU vs MTi-8) and Doppler ego-speed vs MTi
speed (radar), per whole session and per window (drift check). Subtract a
sensor's offset from its timestamps to correct it. Low-confidence results
(`[LOW]`) mean the drive lacked excitation (use accelerations and turns, not
a constant-speed straight line). A large window-to-window spread is jitter
that a constant offset cannot remove. Library code: `fusion/timesync.py`.

## Recording is per-sensor and native-rate

Each sensor has its own recorder(s), started/stopped together by
`run_gcs.py`'s `on_start()`/`on_stop()`, but running completely independently
of one another once started:

| Sensor | Raw/native recorder | Structured recorder | Independent of camera? |
|--------|---------------------|----------------------|-------------------------|
| ZED 2 | SVO2 (`camera.start_recording`) | depth/disparity/confidence NPY, `metadata/frames` | Camera *is* its own native rate |
| MTi-8 | raw Xbus (`mti.start_raw_log`) | `MTiStructuredPoller` → HDF5, at ~200 Hz | Yes |
| Radar (AWR1843) | ROS1 `.bag` (`RadarBagPoller`) | `RadarStructuredPoller` → JSON/heatmap/NPY + `metadata/radar` | Yes |
| Lidar (Ouster) | one pcap + one config JSON (`lidar/receiver.py`, always independent) | scans decoded from the pcap offline, on demand (`dataset.reader.iter_lidar_frames`) — never pre-generated | Yes |
| GNSS | raw NMEA log (`gnss.start_raw_log`) | `GNSSStructuredPoller` → `metadata/gnss` | Yes |

See `DATASET_WRITER.md` for exact formats and paths.

Ingestion detail for the two tightest-coupled sensors:

- **MTi-8 (tight coupling).** `MTiReceiver` runs a daemon thread: serial
  bytes → `XbusParser` → MTData2 decode → monotonic stamp → `MTiValidator` →
  ring buffer (`deque(maxlen=300)`, ≈1.5 s at 200 Hz) guarded by a lock. The
  validator (`mti8/validator.py`) gates what enters the buffer: NaN/Inf
  rejection on accel/gyro/mag/quaternion, quaternion norm check, packet-
  counter gap tracking, and a rotation-discontinuity check (logged as
  *suspicious*, not rejected). Only valid samples land in the buffer, so the
  matcher never sees garbage.
- **GNSS (loose coupling).** `GNSSReceiver` parses NMEA GGA/RMC in its own
  thread and keeps only the **latest** fix. It is *not* time-matched:
  `Synchronizer._get_gnss()` snapshots whatever fix is current when a frame
  passes the sync gate and attaches it to every `SyncedFrame`. RMC UTC is
  converted to GPS week/TOW for downstream use. GNSS is metadata, not
  synchronized navigation data.

## Live/preview matching (`Synchronizer.sync()`)

```
                 ┌──────────────────┐   serial/Xbus    ┌─────────────────────┐
   MTi-8  ──────▶│  MTiReceiver     │──validated──────▶│ ring buffer         │
   200 Hz        │  (thread)        │  samples         │ deque(maxlen=300)   │
                 └──────────────────┘                  └──────────┬──────────┘
                                                                  │ get_nearest()
                 ┌──────────────────┐                             │
   ZED 2  ──────▶│  ZEDCamera.grab()│── Frame ──┐                 │
   ~15–30 fps    └──────────────────┘           ▼                 ▼
                                              ┌─────────────────────────┐
                 ┌──────────────────┐         │     Synchronizer.sync() │
   GNSS   ──────▶│  GNSSReceiver    │─latest──▶│  match + tolerance    │
   1–10 Hz       │  (thread)        │  fix     └───────────┬─────────────┘
                 └──────────────────┘                      │ SyncedFrame | None
                                                           ▼
                                            AcquisitionThread ──▶ DatasetWriter
                                                           │
                                                     SharedFrame snapshot
                                                           ▼
                                                      Qt dashboard
```

`AcquisitionThread` calls `Synchronizer.sync(camera_frame)` on every camera
grab, purely to compute a **dashboard preview** (`sync_error_ms`, live
IMU/radar/lidar overlay values) and to fill two informational columns
(`nearest_mti_timestamp_ns`, `sync_delta_ns`) on the camera's own
`metadata/frames` row. Its result is never checked before writing anything —
`AcquisitionThread` calls `writer.write_camera_frame(frame, synced)` for
every recorded camera frame unconditionally, `synced` or not.

Steps inside `sync()`:

1. **Nearest-neighbour lookup.** `MTiReceiver.get_nearest(target_ts)` scans
   the ring buffer newest-first and stops as soon as `abs(delta)` stops
   improving (the buffer is chronologically ordered, so the first local
   minimum is the global one). Returns the IMU sample closest in time to the
   camera frame.
2. **Acceptance gate.** If no IMU sample exists, or `delta =
   |camera_timestamp_ns - imu_timestamp_ns|` exceeds the tolerance below, the
   frame is **rejected for preview purposes only**: `sync()` returns `None`
   and the dashboard's sync error shows the `999.0 ms` sentinel. The camera
   frame is still recorded — a rejected preview match never prevents
   `AcquisitionThread` from writing it.
3. **Radar/lidar attach.** Same nearest-neighbour + tolerance pattern,
   independently, against their own ring buffers.
4. **GNSS attach.** Latest fix snapshot is attached (always succeeds, even
   with no fix — fields default to zero / `"No Fix"`).
5. **Emit.** A `SyncedFrame` carrying all raw payloads plus each pair's
   timestamps/deltas. The acquisition thread assigns a fresh `frame_uuid`.

Live matching tolerances (`config/settings.py`), used only for this preview:

| Pair | Method | Tolerance | Default |
|------|--------|-----------|---------|
| Camera ↔ IMU (MTi-8) | `get_nearest` | `MAX_DELTA_NS` | 20 ms |
| Camera ↔ Radar (AWR1843) | `get_nearest` | `RADAR_MAX_DELTA_NS` | 80 ms |
| Camera ↔ Lidar (Ouster) | `get_nearest` | `LIDAR_MAX_DELTA_NS` | 50 ms |
| GNSS | `latest()` (no match, not time-gated) | — | — |

Why 20 ms for IMU: at 15–30 fps the frame period is 33–66 ms, so ±20 ms
guarantees at most one IMU-mismatched frame boundary; the MTi-8's 200 Hz rate
means the typical residual is well under 2.5 ms.

`Synchronizer.latest_lidar_frame()` bypasses even this preview gate for the
live lidar dashboard view.

### Failure modes (preview only — none of these affect recording)

| Condition                     | Behaviour                                    |
|-------------------------------|----------------------------------------------|
| IMU disconnected / no packets | `get_nearest` → None → preview shows unsynced |
| Delta exceeds tolerance (sensor stall) | preview shows unsynced until stream recovers |
| Write queue full              | frame counted as `dropped` by the writer     |

A "rejected" preview match is simply skipped for the dashboard's residual
display — there is no interpolation or back-filling, and it has no bearing
on whether the camera frame, or any other sensor's sample, was recorded.

## Offline synchronization (`fusion/offline_sync.py`)

Run **after** recording stops (`synchronize-dataset <dataset_root>` CLI, or
`fusion.offline_sync.synchronize_session(root)`), this reads the independent
native-rate streams a session already wrote — `metadata/frames` (camera),
`sensors/*.h5` (MTi), `metadata/radar` (radar), `metadata/gnss` (GNSS), and
`<session_id>/raw/lidar.pcap` + `<session_id>/raw/lidar_0.json` (lidar — decoded on the fly, there is
no per-frame lidar file) — and for every camera frame finds the nearest
sample in each other modality via `np.searchsorted` over the complete
recorded arrays, plus the residual:

```
Δt_s = t_s - t_camera
```

Each row of the output (`metadata/synchronization/*.parquet`) carries
`{modality}_timestamp_ns`, `dt_camera_{modality}_ns`, and
`{modality}_in_tolerance` (bool, using the same tolerances as the live
preview, plus `GNSS_MAX_DELTA_NS`). Rows outside tolerance are kept and
flagged `False`, never dropped — synchronization quality is auditable, not
hidden. Re-running this over the same session only appends more Parquet
parts; it never modifies or deletes the native per-sensor streams it reads.

## `SyncedFrame` (live-preview container only)

`fusion/synced_frame.py` — used exclusively by `Synchronizer.sync()` for the
dashboard preview described above; it has no role in what actually reaches
disk any more.

| Field | Type | Meaning |
|-------|------|---------|
| `camera` | `dataset.Frame` | source camera frame |
| `imu` | `mti8.MTiFrame` | nearest validated MTi-8 sample |
| `camera_timestamp_ns` / `imu_timestamp_ns` / `delta_ns` | int | preview match |
| `gnss` | `GNSSData` | latest GNSS fix (best-effort) |
| `radar` / `lidar` | `RadarData` / `LidarData` | matched preview data (empty when unmatched) |
| `frame_uuid` | str | unique id set by `AcquisitionThread` |

## Dashboard visibility

The GUI never touches the acquisition path; it reads a lock-free
`SharedFrame` snapshot refreshed by `AcquisitionThread`.

- `sync_error_ms` — current residual (`999.0` = last frame unsynced)
- `SyncDiagnosticsPanel` (`dashboard/sync_panel.py`) — current/avg/min/max,
  p50/p95/p99, worst case, rejected percentage, scrolling history and
  histogram, with reference lines at **15 ms (warn)** and **20 ms (reject)**.
- Pre-record health gate (`dashboard/gate.py`): the `Synchronizer`/radar/lidar
  checks are informational only and never block recording — only
  camera/IMU/disk/calibration/settings checks are critical.

## Thread model summary

| Thread             | Role                                                        |
|--------------------|-------------------------------------------------------------|
| MTi receiver       | serial → parse → validate → buffer                          |
| Radar receiver     | serial → TLV parse → buffer                                 |
| Lidar receiver     | Ouster SDK ingest → buffer + pcap/JSON (always, own thread) |
| GNSS receiver      | serial NMEA → latest fix + buffer (+ NTRIP corrections out-of-band) |
| AcquisitionThread  | grab → color convert → sync (preview) → enqueue camera write → publish snapshot |
| MTi/Radar/GNSS structured pollers | drain buffers at native rate → enqueue writes |
| Writer thread(s)   | NPY/HDF5/JSON/Parquet I/O off every hot path                |
| Qt main thread     | renders the latest snapshot only                            |

Synchronization itself is lock-free from the acquisition side: it takes one
short lock-held scan of each ring buffer per frame and never blocks on I/O.

## Drop / gap bookkeeping

Packet/frame sequence gaps are counted, never silently absorbed:

- MTi: `MTiValidator.counters.dropped_packets`, from `packet_counter` gaps.
- Radar: `RadarReceiver.dropped_frames`, from the sensor's own
  `frame_number` header sequence.
- Lidar: `LidarReceiver.dropped_frames`, from `frame_id` sequence gaps.

`run_gcs.py`'s `on_stop()` records all three as `EventWriter` events at the
end of each session.

## Known limits

- Arrival-time stamping includes transport jitter; no hardware sync between
  ZED and MTi, and no PTP or GPS PPS is implemented today. An upgrade path
  would be PTP or a shared strobe/PPS — the timestamp architecture already
  separates device-clock from host-clock fields (see `DATASET_FORMAT.md`)
  so adding a PPS input would not require a redesign.
- GNSS is latest-value only for this live preview — adequate for geo-tagging,
  not for tightly coupled fusion. (The recorded GNSS stream, independent of
  this preview, does buffer every fix at native rate — see
  `DATASET_WRITER.md`.)
- Ring buffer covers ≈1.5 s; an IMU outage longer than that yields unmatched
  preview frames until the buffer refills. This has no effect on recording.
