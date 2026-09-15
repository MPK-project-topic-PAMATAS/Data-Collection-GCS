# Synchronization — Ground Control Station

> **This document describes the live dashboard preview only.** Recording is
> no longer gated on synchronization — every sensor (camera, MTi, radar,
> lidar, GNSS) is recorded independently at its own native rate regardless of
> whether it matches any other sensor. The authoritative description of the
> current architecture, including the offline/derived synchronization step,
> is `docs/SYNCHRONIZATION.md`; this file covers only how the live dashboard
> computes its preview residual (`SyncedFrame`, `Synchronizer.sync()`).

How the GCS aligns the ZED 2 stereo camera, the MTi-8 IMU, and the GNSS
receiver into a single preview data stream (`SyncedFrame`) for dashboard
display — a separate, later, offline step aligns the actual recorded data
(see `docs/SYNCHRONIZATION.md`).

## Overview

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
   GNSS   ──────▶│  GNSSReceiver    │─latest──▶│  match + 20 ms gate   │
   1–10 Hz       │  (thread)        │  fix     └───────────┬─────────────┘
                 └──────────────────┘                      │ SyncedFrame | None
                                                           ▼
                                            AcquisitionThread ──▶ DatasetWriter
                                                           │
                                                     SharedFrame snapshot
                                                           ▼
                                                      Qt dashboard
```

Camera drives the pipeline. Every camera grab triggers one synchronization
attempt; IMU and GNSS run on their own background threads and are read,
never waited on.

## Time base

**All matching happens in the monotonic clock domain (`time.monotonic_ns()`).**
There is no PTP and no hardware PPS/trigger implemented anywhere in this
codebase — see `docs/SYNCHRONIZATION.md` for the full clock-domain
discussion, including the MTi's own device-clock fields
(`sample_time_fine`/`sample_time_coarse`) which are preserved but not used
for matching.

| Source      | Stamp taken at                                   | Where                              |
|-------------|--------------------------------------------------|------------------------------------|
| ZED frame   | immediately after a successful SDK grab          | `camera/zed_camera.py` (`grab()`)  |
| MTi sample  | packet arrival time, after Xbus decode           | `mti8/receiver.py` (`_loop()`)     |
| GNSS fix    | arrival time of last NMEA sentence (kept in fix dict) | `gnss/receiver.py`            |

Each camera frame additionally records `wall_timestamp_ns` (`time.time_ns()`).
The monotonic domain is used because it is immune to NTP/UTC step changes;
the wall-clock pair is used later to translate matched IMU stamps back to UTC
for persistence (see [Persistence](#persistence)).

There is **no hardware trigger or PTP** between devices — this is software
timestamping at arrival. The 20 ms acceptance window absorbs the resulting
USB/serial jitter.

## Sensor ingestion

### MTi-8 (tight coupling)

`MTiReceiver` runs a daemon thread: serial bytes → `XbusParser` → MTData2
decode → monotonic stamp → `MTiValidator` → ring buffer
(`deque(maxlen=300)`, ≈1.5 s at 200 Hz) guarded by a lock.

The validator (`mti8/validator.py`) gates what enters the buffer:

- NaN/Inf rejection on accel/gyro/mag/quaternion
- quaternion norm check (unit within tolerance)
- packet-counter gap tracking (counts dropped sensor samples)
- rotation-discontinuity check (logged as *suspicious*, not rejected)

Only valid samples land in the buffer, so the matcher never sees garbage.

### GNSS (loose coupling)

`GNSSReceiver` parses NMEA GGA/RMC in its own thread and keeps only the
**latest** fix. It is *not* time-matched: `Synchronizer._get_gnss()` snapshots
whatever fix is current when a frame passes the sync gate and attaches it to
every `SyncedFrame`. RMC UTC is converted to GPS week/TOW for downstream use.
GNSS is metadata, not synchronized navigation data.

## The matching algorithm

`fusion/synchronizer.py` — `Synchronizer.sync(camera_frame)`:

1. **Nearest-neighbour lookup.** `MTiReceiver.get_nearest(target_ts)` scans
   the ring buffer newest-first and stops as soon as `abs(delta)` stops
   improving (the buffer is chronologically ordered, so the first local
   minimum is the global one). Returns the IMU sample closest in time to the
   camera frame.
2. **Acceptance gate.** If no IMU sample exists, or

   ```
   delta = |camera_timestamp_ns - imu_timestamp_ns|
   delta > MAX_DELTA_NS = 20_000_000   # 20 ms
   ```

   the frame is **rejected for preview purposes**: `sync()` returns `None`
   and the dashboard's sync error shows the `999.0 ms` sentinel. The camera
   frame is still recorded — `sync()` returning `None` no longer prevents
   `AcquisitionThread` from writing it (see `docs/SYNCHRONIZATION.md`).
3. **GNSS attach.** Latest fix snapshot is attached (always succeeds, even
   with no fix — fields default to zero / `"No Fix"`).
4. **Emit.** A `SyncedFrame` carrying both raw payloads plus
   `camera_timestamp_ns`, `imu_timestamp_ns`, `delta_ns` and the GNSS block.
   The acquisition thread assigns a fresh `frame_uuid`.

Why 20 ms: at 15–30 fps the frame period is 33–66 ms, so ±20 ms guarantees
at most one IMU-mismatched frame boundary; the MTi-8's 200 Hz rate means the
typical residual is well under 2.5 ms.

### Failure modes (preview only — none of these affect recording)

| Condition                     | Behaviour                                    |
|-------------------------------|----------------------------------------------|
| IMU disconnected / no packets | `get_nearest` → None → preview shows unsynced |
| Delta > 20 ms (IMU stall)     | preview shows unsynced until stream recovers |
| Write queue full              | frame counted as `dropped` by the writer     |

A "rejected" preview match is simply skipped for the dashboard's residual
display — there is no interpolation or back-filling, and it has no bearing
on whether the camera frame, or any other sensor's sample, was recorded.

## Persistence

Recording is per-sensor now — see `docs/DATASET_WRITER.md` for exactly what
`dataset/writer.py` writes for each of camera/MTi/radar/GNSS, and
`docs/SYNCHRONIZATION.md` for how the actual recorded pairing is
reconstructed **offline** (`fusion/offline_sync.py`), independent of this
live preview. The MTi's HDF5 record preserves its own `device_timestamp`
(`sample_time_fine`/`sample_time_coarse`) and `packet_counter` alongside the
host `timestamp_ns` — that field-level detail lives in `docs/DATASET_FORMAT.md`.

## Dashboard visibility

The GUI never touches the acquisition path; it reads a lock-free
`SharedFrame` snapshot refreshed by `AcquisitionThread`.

- `sync_error_ms` — current residual (`999.0` = last frame unsynced)
- `SyncDiagnosticsPanel` (`dashboard/sync_panel.py`) — current/avg/min/max,
  p50/p95/p99, worst case, rejected percentage, scrolling history and
  histogram, with reference lines at **15 ms (warn)** and **20 ms (reject)**.
- Pre-record health gate (`dashboard/gate.py`): the `Synchronizer`/radar/lidar
  checks are informational only and never block recording — only
  camera/IMU/disk/calibration/settings checks are critical. See
  `docs/SYNCHRONIZATION.md`.

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
short lock-held scan of the IMU buffer per frame and never blocks on I/O.

## Known limits

- Arrival-time stamping includes transport jitter; no hardware sync between
  ZED and MTi, and no PTP or GPS PPS is implemented today. An upgrade path
  would be PTP or a shared strobe/PPS — the timestamp architecture already
  separates device-clock from host-clock fields (see `docs/DATASET_FORMAT.md`)
  so adding a PPS input would not require a redesign.
- GNSS is latest-value only for this live preview — adequate for geo-tagging,
  not for tightly coupled fusion. (The recorded GNSS stream, independent of
  this preview, does buffer every fix at native rate — see
  `docs/DATASET_WRITER.md`.)
- Ring buffer covers ≈1.5 s; an IMU outage longer than that yields unmatched
  preview frames until the buffer refills. This has no effect on recording.
