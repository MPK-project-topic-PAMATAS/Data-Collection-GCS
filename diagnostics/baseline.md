# Baseline — Codebase Subsystem Map

Generated: 2026-07-27

## ZED Camera Subsystem

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 1 | ZED init | `camera/zed_camera.py:22-53` | `ZEDCamera.__init__()` — creates `sl.Camera`, `InitParameters`, `RuntimeParameters`, pre-allocates `sl.Mat` for left/right/depth/conf/disp/view_depth/view_conf |
| 2 | Camera open | `camera/zed_camera.py:55-86` | `open()` — opens camera, extracts calibration (fx, fy, cx, cy, baseline, distortion), prints config |
| 3 | Frame grab | `camera/zed_camera.py:164-229` | `grab()` — retrieves left/right images, depth/confidence/disparity measures, depth/confidence views, optional 180° flip, returns `Frame` |
| 4 | Depth retrieval | `camera/zed_camera.py:175` | `retrieve_measure(depth_mat, sl.MEASURE.DEPTH)` — float32 meters |
| 5 | Disparity retrieval | `camera/zed_camera.py:177` | `retrieve_measure(disp_mat, sl.MEASURE.DISPARITY)` — float32, **sign convention unknown** |
| 6 | Disparity diag | `camera/zed_camera.py:94-121` | `_log_disparity_diagnostics()` — runs once on first grab, prints shape/dtype/finite/NaN/Inf/min/max/median |
| 7 | Confidence retrieval | `camera/zed_camera.py:176` | `retrieve_measure(conf_mat, sl.MEASURE.CONFIDENCE)` — uint8 |
| 8 | Depth view | `camera/zed_camera.py:180` | `retrieve_image(view_depth_mat, sl.VIEW.DEPTH)` — uint8 BGRA, colorized by SDK |
| 9 | Confidence view | `camera/zed_camera.py:181` | `retrieve_image(view_conf_mat, sl.VIEW.CONFIDENCE)` — uint8 |
| 10 | Calibration | `camera/zed_camera.py:64-84` | `self.calib` dict with fx=521.92, baseline=0.1196, etc. |
| 11 | Config | `config/settings.py:1-41` | All constants: NEURAL mode, HD720@30, 0.3-30m, confidence=100 |

## Disparity Visualization

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 12 | `_apply_disparity_colormap` | `dashboard/camera_panel.py:265-280` | Normalizes float32→uint8 with percentile clipping, applies Plasma colormap. **Filters `d > 0`** — will produce black if SDK returns negative disparity |
| 13 | `_compute_disparity` | `dashboard/camera_panel.py:283-291` | Fallback: computes disparity from depth via `d = f*B/Z` |
| 14 | CameraPanel.update_frames | `dashboard/camera_panel.py:186-236` | Routes disparity to bottom-right slot |

## Dataset Writer

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 15 | Writer.write | `dataset/writer.py:304-417` | Writes `cam.disparity.astype(np.float32)` as raw `.npy` — no modification |
| 16 | Frame dataclass | `dataset/frame.py:1-21` | Holds `disparity: np.ndarray = None` |

## MTi-8 IMU Subsystem

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 17 | Serial reader | `mti8/serial_reader.py:13-130` | Threaded serial I/O, auto-detect port, reconnect, raw log |
| 18 | XBus parser | `mti8/xbus_parser.py:18-81` | Buffer-based XBus framing: preamble(0xFA)→BID(0xFF)→MID→LEN→payload→checksum |
| 19 | MTData2 decoder | `mti8/mtdata2_decoder.py:59-143` | XDI packet walker: 2-byte data_id + 1-byte size + data, big-endian |
| 20 | MTiFrame | `mti8/frame.py:6-48` | Dataclass: quaternion(xyzw), acceleration, gyroscope, magnetometer, GNSS, status |
| 21 | Receiver | `mti8/receiver.py:9-98` | Orchestrates serial→parser→decoder, nearest-frame lookup for sync |
| 22 | Verify tool | `mti8/verify.py` | CLI: prints all XDI fields, packet loss, quaternion norm for MT Manager comparison |

## Synchronization

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 23 | Synchronizer | `fusion/synchronizer.py:8-56` | Matches camera frame to nearest IMU frame within 20ms window |
| 24 | SyncedFrame | `fusion/synced_frame.py:1-33` | Dataclass: camera + imu + gnss + timestamps |

## Dashboard

| # | Function | File:Line | Description |
|---|----------|-----------|-------------|
| 25 | SharedFrame | `dashboard/acquisition.py:19-80` | Lock-free buffer: ~37 slots for all live telemetry |
| 26 | AcquisitionThread | `dashboard/acquisition.py:90-264` | grab→sync→write loop, computes depth metrics |
| 27 | MainWindow | `dashboard/main_window.py:34-284` | 3-tier Qt layout, timer-driven updates |
| 28 | IMUPanel | `dashboard/imu_panel.py` | Displays MTi-8 vs ZED IMU with delta |

## Known Issues (pre-fix)

1. **Disparity visualization black**: `_apply_disparity_colormap` filters `d > 0`. ZED SDK may return negative disparity values (negative convention), causing all valid pixels to be discarded → black image.
2. **Disparity sign convention**: Not yet validated. Need to check ZED SDK documentation or measure empirically.
3. **MTi-8 parser**: No identified bugs yet — needs Phase 5-8 audit.
