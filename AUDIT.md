# PHASE 0 AUDIT — CURRENT STORAGE PIPELINE

## CURRENT STORAGE

| File | Format | Producer | Consumer |
|------|--------|----------|----------|
| `dataset_YYYYMMDD_HHMMSS/images/left/{frame_id:06d}.jpg` | JPEG (configurable PNG/WEBP) | `DatasetWriter._write_loop` | `playback.py`, `validate_dataset.py`, `calibration/convert_for_kalibr.py`, Dashboard |
| `dataset_YYYYMMDD_HHMMSS/images/right/{frame_id:06d}.jpg` | JPEG | `DatasetWriter._write_loop` | Same as left |
| `dataset_YYYYMMDD_HHMMSS/measurements/depth/{frame_id:06d}.npy` | NPY (float32) | `DatasetWriter._write_loop` | `playback.py`, `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/measurements/disparity/{frame_id:06d}.npy` | NPY (float32) | `DatasetWriter._write_loop` | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/measurements/confidence/{frame_id:06d}.npy` | NPY (float32) | `DatasetWriter._write_loop` | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/timestamps.csv` | CSV | `DatasetWriter._write_loop` (via `_ensure_timestamps_csv`) | `playback.py`, `validate_dataset.py`, `calibration/convert_for_kalibr.py` |
| `dataset_YYYYMMDD_HHMMSS/imu/mti.csv` | CSV | `DatasetWriter._write_loop` (via `_ensure_mti_csv`) | `playback.py`, `validate_dataset.py`, `calibration/convert_for_kalibr.py` |
| `dataset_YYYYMMDD_HHMMSS/imu/zed.csv` | CSV | `DatasetWriter._write_loop` (via `_ensure_zed_imu_csv`) | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/gnss/gnss.csv` | CSV | `DatasetWriter._write_loop` (via `_ensure_gnss_csv`) | `playback.py`, `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/metadata/camera_metadata.csv` | CSV | `DatasetWriter._write_loop` (via `_ensure_cam_csv`) | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/metadata/statistics.csv` | CSV | `DatasetWriter.save_manifest` (via `_ensure_stats_csv`) | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/events.csv` | CSV | `DatasetWriter.add_event` (via `_ensure_events_csv`) | `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/dataset.json` | JSON | `DatasetWriter.save_manifest` | `validate_dataset.py`, `calibration/convert_for_kalibr.py` |
| `dataset_YYYYMMDD_HHMMSS/calibration/zed_intrinsics.yaml` | YAML | `DatasetWriter.save_calibration` | `calibration/convert_for_kalibr.py`, `validate_dataset.py` |
| `dataset_YYYYMMDD_HHMMSS/calibration/camera_info.json` | JSON | `DatasetWriter.save_calibration` | `calibration/convert_for_kalibr.py` |
| `dataset_YYYYMMDD_HHMMSS/extrinsics/*.yaml` | YAML | `DatasetWriter.save_calibration` (placeholders) | `calibration/convert_for_kalibr.py` |
| `dataset_YYYYMMDD_HHMMSS/raw/recording.svo2` | SVO2 (ZED native) | `camera.start_recording` (ZED SDK) | ZED SDK playback |
| `dataset_YYYYMMDD_HHMMSS/raw/mti.xbus` | Raw Xbus binary | `MTiReceiver.start_raw_log` | Raw decoder |

---

## ARCHITECTURE COMPONENTS

### 1. DatasetWriter (`dataset/writer.py:55-498`)
- Creates `dataset_YYYYMMDD_HHMMSS/` directory per instantiation
- Owns a bounded `queue.Queue(maxsize=128)` + dedicated writer thread (`_write_loop`)
- Opens CSV file handles lazily on first write (`_ensure_*_csv`)
- Writes images via `cv2.imwrite`, depth/disparity via `np.save`
- **Critical**: All CSV writes happen in the writer thread (not camera thread) — preserves async architecture

### 2. Async Writer Queue
- `self._queue = queue.Queue(maxsize=128)` (line 84)
- `write()` enqueues `SyncedFrame`, returns immediately
- `_write_loop` dequeues, writes all CSVs + images + NPY, calls `task_done()`

### 3. Camera Frame Object (`dataset/frame.py:6-21`)
```python
@dataclass
class Frame:
    frame_id: int
    timestamp_ns: int          # monotonic_ns from grab()
    left: np.ndarray           # BGR
    right: np.ndarray          # BGR
    depth: np.ndarray          # float32 raw measure
    zed_imu: dict              # ZED IMU (diagnostic only)
    disparity: np.ndarray = None
    confidence: np.ndarray = None
    view_depth: np.ndarray = None
    view_confidence: np.ndarray = None
    exposure: float = 0.0
    gain: float = 0.0
    temperature: float = 0.0
    pose: Optional[dict] = None
```

### 4. Synchronized Camera/MTi Object (`fusion/synced_frame.py:24-33`)
```python
@dataclass
class SyncedFrame:
    camera: Frame
    imu: MTiFrame
    camera_timestamp_ns: int
    imu_timestamp_ns: int
    delta_ns: int
    gnss: GNSSData = field(default_factory=GNSSData)
    gnss_timestamp_ns: int = 0
    frame_uuid: str = ""
```

### 5. Output Directory Creation (`writer.py:56-82`)
```python
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
self.root = Path(root) / f"dataset_{timestamp}"
```
→ **Creates new directory per DatasetWriter instantiation** (i.e., per process run, not per REC/STOP)

### 6. PNG/JPEG Writing (`writer.py:310-315`)
```python
params = _IMAGE_PARAMS.get(self._image_ext, JPEG_PARAMS)
cv2.imwrite(left_path, cam.left, params)
cv2.imwrite(right_path, cam.right, params)
```

### 7. NPY Writing (`writer.py:320-335`)
```python
np.save(depth_path, cam.depth.astype(np.float32))
np.save(disp_path, cam.disparity.astype(np.float32))
np.save(conf_path, cam.confidence.astype(np.float32))
```

### 8. CSV Writing — MTi (`writer.py:350-364`)
```python
self._ensure_mti_csv()
self._mti_writer.writerow([
    imu.timestamp_ns, imu.packet_counter,
    a[0], a[1], a[2], g[0], g[1], g[2],
    m[0], m[1], m[2], q[0], q[1], q[2], q[3],
    imu.status_word,
])
```
→ **Primary MTi storage is CSV** — violates requirement #1

### 9. CSV Writing — ZED IMU (`writer.py:366-380`)
```python
self._ensure_zed_imu_csv()
self._zed_imu_writer.writerow([...])
```

### 10. CSV Writing — Timestamps (`writer.py:337-348`)
```python
self._ensure_timestamps_csv()
self._ts_writer.writerow([
    cam.frame_id, synced.frame_uuid,
    cam.timestamp_ns, imu.timestamp_ns,
    zed_imu_ts, gnss_ns, round(synced.delta_ns / 1e6, 3),
])
```

### 11. CSV Writing — GNSS/Camera/Events/Stats (`writer.py:382-437`)

### 12. MTi Logging (`mti8/receiver.py:36-59`)
- Serial thread reads Xbus packets → `decode()` → `MTiFrame`
- Stores in ring buffer (`deque(maxlen=300)`) + `self._latest`
- `get_nearest(target_ts)` does linear search from newest backward
- **No disk writes in serial thread** — correct architecture

### 13. ZED IMU Logging (`camera/zed_camera.py:231-261`)
- `_get_imu()` called in `grab()` — returns dict with accel/gyro/quat/mag/temp/timestamp
- Stored in `Frame.zed_imu` → written to CSV by writer thread

### 14. Session Creation
- **No explicit session concept** — each `DatasetWriter` = one dataset directory
- REC/STOP in dashboard only toggles `writer.write()` calls and SVO/raw logging
- `run_gcs.py:106-136` — `on_start`/`on_stop` control recording flag + SVO + raw IMU

### 15. Frame IDs
- `ZEDCamera.frame_id` increments from 0 per camera open (`zed_camera.py:48, 227`)
- Resets on camera reopen

### 16. Timestamp Generation
- Camera: `time.monotonic_ns()` at grab (`zed_camera.py:168`)
- MTi: `time.monotonic_ns()` at packet decode (`receiver.py:50`)
- GNSS: from receiver timestamp
- **All nanosecond, monotonic clock** — correct authoritative source

### 17. Dashboard References (`dashboard/acquisition.py`, `recorder_panel.py`)
- `AcquisitionThread` holds `writer` reference, reads `writer._queue.qsize()`, `writer.bytes_written`, `writer._dropped`
- `RecorderPanel` displays stats from `SharedFrame` populated by acquisition thread
- `MainWindow` triggers `writer` via `acq.set_recording(True/False)`

### 18. Calibration Scripts Expecting Old Structure
- `calibration/convert_for_kalibr.py:32-69` — expects `images/left/`, `images/right/`, `imu/mti.csv`, `metadata/`
- `calibration/export_zed_calibration.py` — reads/writes calibration YAML/JSON

### 19. Kalibr Converter Assumptions
- Loads `timestamps.csv` for frame timestamps
- Loads `imu/mti.csv` for IMU data (per-frame, not full-rate)
- Expects `.jpg` images named by frame_id
- Generates `imu0.csv` at camera frame rate (downsampled from MTi 200Hz)

### 20. Diagnostic Scripts Reading CSV
- `validate_dataset.py` — reads ALL CSV files for validation
- `playback.py` — loads `timestamps.csv`, `imu/mti.csv`, `gnss/gnss.csv` into dicts
- `diagnostics/imu_alignment_test.py` — reads MTi CSV for alignment analysis

---

## DEPENDENCY MAP

```
main.py / run_gcs.py
    └─> DatasetWriter (creates dataset_TIMESTAMP/)
            ├─> Writer thread (_write_loop)
            │     ├─> cv2.imwrite (images)
            │     ├─> np.save (depth/disparity/confidence)
            │     ├─> CSV writers (timestamps, mti, zed_imu, gnss, cam_meta, events, stats)
            │     └─> JSON/YAML (manifest, calibration)
            └─> Writer queue ← AcquisitionThread.write()

AcquisitionThread (dashboard/acquisition.py)
    ├─> ZEDCamera.grab() → Frame
    ├─> Synchronizer.sync() → SyncedFrame
    └─> DatasetWriter.write(SyncedFrame)

MTiReceiver (mti8/receiver.py)
    ├─> Serial thread → MTiFrame (ring buffer)
    └─> get_nearest(ts) → Synchronizer

playback.py / validate_dataset.py / calibration/convert_for_kalibr.py
    └─> READ: timestamps.csv, mti.csv, zed.csv, gnss.csv, images/**, measurements/**, dataset.json
```

---

## CRITICAL FINDINGS FOR REFACTOR

1. **MTi CSV is primary storage** — must migrate to HDF5 (Phase 4)
2. **Per-run dataset directory** — must become continuous namespace (Phase 1)
3. **Frame naming `frame_id:06d`** — must become `YYYYMMDD_HHMMSS_nnnnnnnnn_frameid` (Phase 2)
4. **No session concept** — REC/STOP only gates writing, doesn't create sessions (Phase 13)
5. **Kalibr converter reads CSV** — must support HDF5/Parquet export (Phase 20/21)
6. **validate_dataset.py checks CSV schemas** — must be updated for new format (Phase 30)
7. **playback.py loads CSVs into dict** — must be updated for HDF5/Parquet (Phase 18)
8. **Dashboard reads writer queue stats** — must work with new writer architecture (Phase 22)
9. **Async writer thread pattern preserved** — HDF5/Parquet writes must stay in writer thread (Phase 8/9)
10. **MTi ring buffer + get_nearest** — Synchronizer must continue working unchanged (Phase 4/8)