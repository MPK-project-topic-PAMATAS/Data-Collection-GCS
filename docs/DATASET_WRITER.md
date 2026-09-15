# Dataset Writer — formats per sensor

What the `DatasetWriter` (`dataset/writer.py`) and its companion recorders
write to disk, and in which format. This complements `SYNCHRONIZATION.md`
(how samples are time-aligned — now strictly a derived, offline step) and
`DATASET_FORMAT.md` (the on-disk layout). Every recording session gets its
own self-contained subfolder, `<dataset_root>/<session_id>/` (default
dataset root: `DATASET_DIR`) — every path below (`metadata/...`,
`sensors/...`, `radar/...`, `raw/...`, etc.) is relative to that session's
own subfolder, not the dataset root.

## Recording is per-sensor, not camera-gated

`DatasetWriter` has one independent entry point per sensor — none of them
require a match with any other sensor:

- `write_camera_frame(frame, synced_preview=None)` — always called for every
  recorded camera frame; `synced_preview` (a live `Synchronizer.sync()`
  result, or `None`) only fills two informational preview columns.
- `write_mti_sample(mti_frame, host_rx_timestamp_ns)` — called once per
  **native** MTi packet (~200 Hz), by `MTiStructuredPoller`
  (`dataset/mti_structured_recorder.py`), independent of the camera loop.
- `write_radar_frame(radar_frame)` — called once per native radar frame, by
  `RadarStructuredPoller` (`dataset/radar_structured_recorder.py`).
- `write_gnss_fix(fix)` — called once per parsed GGA/RMC fix, by
  `GNSSStructuredPoller` (`dataset/gnss_structured_recorder.py`).

All four enqueue onto the *same* bounded queue (maxsize 128) that a single
writer thread drains — the "all I/O happens off the camera/sensor threads"
invariant is unchanged, only the trigger for each write is now that sensor's
own arrival, not a cross-sensor match.

Lidar has no `DatasetWriter` method at all: `lidar/receiver.py` writes
**exactly one pcap and one companion config JSON per session** — never one
JSON per frame/scan/packet — from its own ingest thread, independent of
camera sync. That pcap is the primary raw Ouster dataset; the JSON is
session-level sensor configuration/metadata only. Individual scans are never
pre-generated — they're decoded from the pcap offline, on demand, by
`dataset.reader.iter_lidar_frames()`, keyed by the packet-capture timestamps
already recorded inside the pcap itself.

## Recording session lifecycle (`run_gcs.py`)

On **Record** (`on_start`): `writer.begin_session(t0)` opens the logical
session, an immutable `dataset.config_snapshot` is saved
(`metadata/config_snapshot_<session_id>.json` — never contains NTRIP
credentials), then each sensor's raw recorder *and* structured poller are
started. On **Stop** (`on_stop`): all pollers/recorders are stopped, drop/gap
counters are recorded as events, `writer.end_session(t0)` flushes and closes,
then a per-session manifest is written
(`metadata/session_manifest_<session_id>.json`, see
`dataset/session_manifest.py`) listing exactly which files this session
produced — including the single pcap and single config JSON paths — plus
start/stop timing and synchronization tolerances.

Every session writes into its own subfolder, `<dataset_root>/<session_id>/`
(created by `writer.begin_session()`, which also switches `writer.root` and
`writer.raw_dir` to point into it) — a dataset root can hold many sessions'
raw recordings side by side without one overwriting another. This matters:
fixed generic names shared across the whole root would silently overwrite
the previous session's SVO2/pcap/xbus/bag/NMEA the moment a second REC
happens in the same app run (a real bug an external dataset audit found).
Folder isolation, not filename suffixing, is what makes each session's raw
files unique — every raw filename below is plain within its own session's
`raw/` folder.

| Sensor | Raw/native recorder | Format | Structured recorder |
|--------|----------------------|--------|----------------------|
| ZED 2 | `camera.start_recording(svo_path)` → `<session_id>/raw/recording.svo2` | SVO2 | `write_camera_frame` (camera *is* native rate) |
| MTi-8 | `mti.start_raw_log(path)` → `<session_id>/raw/mti.xbus` | raw Xsens Xbus binary | `MTiStructuredPoller` |
| Ouster lidar | `lidar.start_pcap(path)` → **one** `<session_id>/raw/lidar.pcap` + **one** `<session_id>/raw/lidar_0.json` | PCAP + config JSON | none — scans decoded from the pcap offline |
| Radar | `RadarBagPoller` → `<session_id>/raw/radar.bag` | ROS1 bag (points only, see below) | `RadarStructuredPoller` (points + heatmap + range-profile) |
| GNSS | `gnss.start_raw_log(path)` → `<session_id>/raw/gnss.nmea.log` | raw NMEA text | `GNSSStructuredPoller` |

`dataset.reader.resolve_lidar_paths(root, session_id=None)` finds the right
pcap/config-JSON pair for a given session's subfolder, or the most recently
modified session when no session_id is given (`dataset.reader.
list_sessions()`); it also falls back to two now-superseded layouts for
older datasets — a brief interim design that suffixed flat filenames by
session_id, and the original flat `raw/lidar.pcap` from before any session
isolation existed. The exact raw path for every sensor in a given session
is also recorded in that session's own
`<session_id>/metadata/session_manifest_<session_id>.json` — the
authoritative way to look them up rather than reconstructing the pattern.

All raw recorders capture the **full** stream at the sensor's native rate.
Structured recorders now do too (previously MTi/radar/GNSS structured output
was downsampled to, and gated on, the camera↔IMU match — see
`SYNCHRONIZATION.md`).

## Structured (decoded) outputs

Common filename prefix `YYYYMMDD_HHMMSS_NNNNNNNNN_FFFFFFFFFF` (UTC
date·time, `nano_frac` of `timestamp_ns`, frame id) — for human readability
only.

| Modality | Output | Format | Written per |
|----------|--------|--------|-------------|
| Depth | `depth/<prefix>.npy` | float32 `np.save` | camera frame (gated by `SAVE_DEPTH`) |
| Disparity | `disparity/<prefix>.npy` | float32 | camera frame (gated by `SAVE_DISPARITY`) |
| Confidence | `confidence/<prefix>.npy` | float32 | camera frame (gated by `SAVE_CONFIDENCE`) |
| MTi-8 | `sensors/*_mti.h5` (group `/mti`) | HDF5, chunked+lzf | every native MTi packet |
| Radar points | `radar/points/<prefix>.json` | JSON | every native radar frame |
| Radar heatmap | `radar/heatmap/<prefix>.npy` | uint16, shape `(range, Doppler)` | every native radar frame |
| Radar profile | `radar/range_profile/<prefix>.npy` | uint16, shape `(range,)` | every native radar frame (when enabled) |
| Camera frame index | `metadata/frames/date=…/part-*.parquet` | Parquet | one row per camera frame, always |
| Radar frame index | `metadata/radar/date=…/part-*.parquet` | Parquet | one row per native radar frame |
| GNSS fix index | `metadata/gnss/date=…/part-*.parquet` | Parquet | one row per parsed fix |
| Synchronization index | `metadata/synchronization/date=…/part-*.parquet` | Parquet | **derived, offline only** — see `SYNCHRONIZATION.md` |

Lidar has no structured output row of its own — its "index" is the pcap's
own packet timestamps, read directly by `iter_lidar_frames()`/offline sync;
see the dedicated section below.

> Camera images (left/right video + ZED IMU) are **not** written per-frame —
> they live in the SVO2 `<session_id>/raw/recording.svo2`.

### Radar known limitation

Only TLV1 (detected points) and TLV5 (range-Doppler heatmap) are enabled in
the AWR1843's sensor config (`radar/receiver.py:_SENSOR_CONFIG_CMDS`); TLV2
(range profile) is decoded but not enabled by default. **Raw ADC/IQ samples
are not captured** — this driver only consumes the TI mmWave SDK's
post-processed TLV output over the DATA UART, not LVDS/DCA1000 raw capture.
Do not treat TLV detections as raw radar data.

### Radar point-cloud JSON (`radar/points/<prefix>.json`)
```json
{
  "timestamp_ns": 1788501226590041357,   // UTC epoch ns (converted from host
                                          // monotonic via a mono->wall offset
                                          // — see SYNCHRONIZATION.md)
  "frame_id": 42,                         // AWR1843 frame number
  "num_objects": 12,                      // detected-point count
  "points": [ {"x": 1.0, "y": 2.0, "z": 3.0, "velocity_mps": 4.5}, ... ]
}
```
Coordinates are metres; `velocity_mps` is radial velocity.

## Lidar — one pcap + one config JSON (`<session_id>/raw/lidar.pcap`, `<session_id>/raw/lidar_0.json`)

`LidarReceiver.start_pcap()` opens `<session_id>/raw/lidar.pcap` once, at session start,
and its ingest thread expands every incoming scan back into raw UDP packets
(`ouster.sdk.util.frame_to_packets`) appended to that single file for the
whole session — this is the primary raw Ouster dataset. At the same moment,
exactly one sensor configuration/metadata JSON is written to
`raw/lidar_0.json` (`SensorInfo.to_json_string()` — beam angles, calibration,
lidar mode); `run_gcs.py`'s `on_stop()` generates a minimal fallback only if
that write somehow didn't happen, still exactly one file. **No per-frame,
per-scan, or per-packet JSON is ever written.**

Individual scans/point clouds are derived offline, on demand, from this
pcap + JSON pair by `dataset.reader.iter_lidar_frames(pcap_path, meta_path)`:
it opens the pcap via the Ouster SDK's `open_source()` (which auto-discovers
the `_0.json` companion), and for each decoded frame yields a `LiDARFrame`
with `points`/`reflectivity` (from `ChanField.RANGE`/`REFLECTIVITY`) and
`timestamp_ns` taken directly from the pcap's own recorded packet-capture
timestamp (`frame.get_first_valid_packet_timestamp()`) — never fabricated,
never read from a side file. `DatasetReader.lidar_frames()` /
`get_lidar_frame(frame_id)` wrap this for callers that want cached access,
and `fusion/offline_sync.py` uses it directly to source lidar timestamps for
the derived synchronization index.

## Independent radar bag capture — `raw/radar.bag`

Written by `dataset/radar_bag_recorder.py` (`RadarBagRecorder` +
`RadarBagPoller`), a daemon thread polling `radar.get_frame()` and
deduplicating on `timestamp_ns`. Works for both the Python and native radar
backends. A standard ROS1 bag (RoadBag/Autoware-compatible), gated by
`SAVE_RADAR_BAG` (default `True`). Carries points only (not the heatmap —
see the structured `radar/heatmap/*.npy` output above for that). Per frame
it emits two messages:

| Topic | ROS type | Content |
|-------|----------|---------|
| `/radar/points` | `sensor_msgs/PointCloud2` | width=N, fields `x,y,z,velocity_mps` (float32, point_step 16), `frame_id="radar"` |
| `/clock` | `rosgraph_msgs/Clock` | wall time |

Bag time is wall/epoch: `open()` captures `wall_offset = time.time_ns() -
time.monotonic_ns()` and adds it to each frame's `timestamp_ns` for both the
message stamp and the bag record timestamp.

## HDF5 sensor file (`sensors/*_mti.h5`)

Contract: `dataset/hdf5_sensor_writer.py`. Daily-rotated, append-only,
written by a dedicated thread from a thread-safe queue. Full dataset axis
list in `DATASET_FORMAT.md` (`/mti` group). Three timestamp fields per
sample, each with a distinct, documented meaning (see `SYNCHRONIZATION.md`
for the full clock-domain discussion):

- `timestamp_ns` / `host_timestamp_ns` — host `CLOCK_MONOTONIC` at the
  moment the packet was decoded (`MTiReceiver._loop`).
- `device_timestamp` — the MTi's own free-running clock
  (`sample_time_fine | sample_time_coarse << 32`), arbitrary epoch, no
  rollover handling, not synchronized to the host.
- `packet_counter` — the MTi's own packet sequence number, used for drop
  detection (`MTiValidator.counters.dropped_packets`).

## Config snapshot

`dataset/config_snapshot.py: build_config_snapshot()`, written once per
session to `metadata/config_snapshot_<session_id>.json`: effective
camera/MTi/radar/lidar/GNSS configuration, synchronization tolerances, host
clock info (explicitly states no PTP/PPS), git commit, and software version.
Never includes NTRIP username/password — only host/port/mountpoint.

## Concurrency & backpressure

- Writer has a **bounded queue**; every `write_*` method is non-blocking and
  returns immediately. `write_queue_size`, `dropped`, `bytes_written`, and
  HDF5 queue depth are surfaced to the dashboard.
- The writer thread dispatches by tag (`camera`/`mti`/`radar`/`gnss`) to the
  matching `_write_*` method; sensor receivers atomically swap immutable
  frames under their own locks, so serialization never blocks live ingest.

## Key invariants

1. **No silent drops** — queue overflow counted (`_dropped`); packet/frame
   sequence gaps counted per-sensor (`dropped_frames`,
   `MTiValidator.counters.dropped_packets`).
2. **No overwrites** — collision appends `_001`, `_002` suffix.
3. **Never derive timestamps from filenames** — filenames are approximations.
4. **A missing sample from one sensor never removes a sample from another**
   — each `write_*` method is independent; there is no shared gate.
5. **Full-rate raw + full-rate structured coexist** — raw SVO2/Xbus/PCAP/bag
   and the structured HDF5/JSON/NPY/Parquet outputs are both now native-rate;
   synchronization (`metadata/synchronization/`) is a separate, later, purely
   additive step over the recorded data.
6. **Ouster is one pcap + one config JSON per session, never per-frame
   files** — scans are always derived offline from the pcap's own packet
   timestamps, never pre-generated.
