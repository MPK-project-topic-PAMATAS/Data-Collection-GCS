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

## Live/preview matching (`Synchronizer.sync()`)

`AcquisitionThread` still calls `Synchronizer.sync(camera_frame)` on every
camera grab, purely to compute a **dashboard preview** (`sync_error_ms`,
live IMU/radar/lidar overlay values) and to fill two informational columns
(`nearest_mti_timestamp_ns`, `sync_delta_ns`) on the camera's own
`metadata/frames` row. Its result is never checked before writing anything —
`AcquisitionThread` calls `writer.write_camera_frame(frame, synced)` for
every recorded camera frame unconditionally, `synced` or not.

Live matching tolerances (`config/settings.py`), used only for this preview:

| Pair | Method | Tolerance | Default |
|------|--------|-----------|---------|
| Camera ↔ IMU (MTi-8) | `get_nearest` | `MAX_DELTA_NS` | 20 ms |
| Camera ↔ Radar (AWR1843) | `get_nearest` | `RADAR_MAX_DELTA_NS` | 80 ms |
| Camera ↔ Lidar (Ouster) | `get_nearest` | `LIDAR_MAX_DELTA_NS` | 50 ms |
| GNSS | `latest()` (no match, not time-gated) | — | — |

`Synchronizer.latest_lidar_frame()` bypasses even this preview gate for the
live lidar dashboard view, same as before.

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

## Drop / gap bookkeeping

Packet/frame sequence gaps are counted, never silently absorbed:

- MTi: `MTiValidator.counters.dropped_packets`, from `packet_counter` gaps.
- Radar: `RadarReceiver.dropped_frames`, from the sensor's own
  `frame_number` header sequence.
- Lidar: `LidarReceiver.dropped_frames`, from `frame_id` sequence gaps.

`run_gcs.py`'s `on_stop()` records all three as `EventWriter` events at the
end of each session.
