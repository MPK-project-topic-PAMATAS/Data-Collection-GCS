"""Acquisition thread — camera grab, sync, writer in a dedicated thread.

The GUI never touches the acquisition thread. It only reads the latest
frame from a lock-free shared buffer.
"""

from __future__ import annotations
import shutil
import threading
import time
import uuid

import cv2
import numpy as np

from config.settings import DASHBOARD_METRICS_INTERVAL_S
from fusion.synchronizer import Synchronizer
from dataset.writer import DatasetWriter


class SharedFrame:
    """Lock-free latest-frame buffer.  Writers replace, readers copy."""

    __slots__ = (
        "left_rgb", "right_rgb", "depth", "confidence",
        "disparity",
        "view_depth", "view_confidence",
        "valid_depth_pct", "invalid_depth_pct",
        "mean_confidence", "median_depth",
        "timestamp_ns", "frame_id", "frame_uuid", "camera_fps",
        "accel", "gyro", "quat",
        "zed_accel", "zed_gyro", "zed_quat",
        "sync_error_ms", "frame_count", "dropped_frames",
        "bytes_written", "imu_rate", "imu_packets",
        "recording", "recording_duration_s", "dataset_size_mb",
        "disk_free_gb", "write_queue_size", "write_queue_peak",
        "exposure", "gain", "camera_temp",
        "cpu_grab_ns", "cpu_sync_ns", "cpu_write_ns",
        "hdf5_queue_depth", "sensor_queue_peak",
        "mti_samples_written", "mti_drops",
        "session_id", "dataset_root",
        "lat", "lon", "alt", "gnss_fix", "gnss_satellites",
        "gnss_hdop", "gnss_speed", "gnss_course",
        "rtk_fixed", "rtk_float", "ntrip_connected",
        "recording_start", "_lock",
        "mti_stale", "mti_state",
        "zed_imu_stale",
        "radar_points", "radar_range_azimuth", "radar_range_doppler", "radar_ts_ns",
        "radar_frame_id", "radar_num_objects", "radar_stale",
        "radar_range_profile", "radar_streaming", "radar_rate",
        "lidar_points", "lidar_ts_ns", "lidar_frame_id",
        "lidar_num_points", "lidar_streaming", "lidar_rate",
        "lidar_matched", "lidar_sync_delta_ms",
    )

    def __init__(self):
        self._lock = threading.Lock()
        self.left_rgb = None
        self.right_rgb = None
        self.depth = None
        self.confidence = None
        self.disparity = None
        self.view_depth = None
        self.view_confidence = None
        self.valid_depth_pct = 0.0
        self.invalid_depth_pct = 0.0
        self.mean_confidence = 0.0
        self.median_depth = 0.0
        self.timestamp_ns = 0
        self.frame_id = 0
        self.frame_uuid = ""
        self.camera_fps = 0.0
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.quat = [0.0, 0.0, 0.0, 1.0]
        self.zed_accel = [0.0, 0.0, 0.0]
        self.zed_gyro = [0.0, 0.0, 0.0]
        self.zed_quat = [0.0, 0.0, 0.0, 1.0]
        self.sync_error_ms = 0.0
        self.frame_count = 0
        self.dropped_frames = 0
        self.bytes_written = 0
        self.imu_rate = 0.0
        self.imu_packets = 0
        self.recording = False
        self.recording_duration_s = 0.0
        self.dataset_size_mb = 0.0
        self.disk_free_gb = 0.0
        self.write_queue_size = 0
        self.write_queue_peak = 0
        self.exposure = 0.0
        self.gain = 0.0
        self.camera_temp = 0.0
        self.cpu_grab_ns = 0
        self.cpu_sync_ns = 0
        self.cpu_write_ns = 0
        self.hdf5_queue_depth = 0
        self.sensor_queue_peak = 0
        self.mti_samples_written = 0
        self.mti_drops = 0
        self.session_id = ""
        self.dataset_root = ""
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.gnss_fix = 0
        self.gnss_satellites = 0
        self.gnss_hdop = 0.0
        self.gnss_speed = 0.0
        self.gnss_course = 0.0
        self.rtk_fixed = False
        self.rtk_float = False
        self.ntrip_connected = False
        self.recording_start = 0.0
        self.mti_stale = False
        self.mti_state = ""
        self.zed_imu_stale = True
        self.radar_points = []
        self.radar_range_doppler = None
        self.radar_range_azimuth = None
        self.radar_ts_ns = 0
        self.radar_frame_id = 0
        self.radar_num_objects = 0
        self.radar_stale = True
        self.radar_range_profile = None
        self.radar_streaming = False
        self.radar_rate = 0.0
        self.lidar_points = None
        self.lidar_ts_ns = 0
        self.lidar_frame_id = 0
        self.lidar_num_points = 0
        self.lidar_streaming = False
        self.lidar_rate = 0.0
        self.lidar_matched = False
        self.lidar_sync_delta_ms = None

    def update_from(self, other: "SharedFrame"):
        with self._lock:
            for k in self.__slots__:
                if k == "_lock":
                    continue
                setattr(self, k, getattr(other, k))


class AcquisitionThread(threading.Thread):
    """Dedicated thread: grab frames, sync IMU, write to disk.

    The GUI reads from `self.buffer` via snapshot().
    """

    def __init__(self, camera, synchronizer: Synchronizer,
                 writer: DatasetWriter, shared: SharedFrame):
        super().__init__(daemon=True, name="acq")
        self.camera = camera
        self.sync = synchronizer
        self.writer = writer
        self.buffer = shared
        self._running = True
        self._recording = False
        self._frame_count = 0
        self._dropped = 0
        self._imu_count = 0
        self._imu_last_count = 0
        self._imu_last_time = 0.0
        self._bytes_written = 0
        self._t0 = time.monotonic()
        self._last_metrics_time = 0.0
        self._depth_metrics = (0.0, 0.0, 0.0, 0.0)
        self._last_mti_accel = [0.0, 0.0, 0.0]
        self._last_mti_gyro = [0.0, 0.0, 0.0]
        self._last_mti_quat = [0.0, 0.0, 0.0, 1.0]
        self._mti_stale = False
        self._mti_state = ""
        # ZED's onboard IMU is read over a separate USB HID interface from
        # the video feed; that interface has been observed to glitch/
        # re-enumerate on camera open (kernel logs a UVC protocol error
        # followed by a HID re-enumeration), during which get_sensors_data()
        # can keep returning SUCCESS with a stale, non-advancing sample —
        # the dashboard would otherwise show a frozen reading with no
        # indication anything is wrong. Detect it via the ZED SDK's own
        # per-sample IMU timestamp instead of trusting the SUCCESS code.
        self._last_zed_imu_ts = 0
        self._last_zed_imu_change_time = time.monotonic()

    def stop(self):
        self._running = False

    def set_recording(self, recording: bool):
        self._recording = recording

    def _read_zed_imu(self, zed_imu: dict | None):
        """Extract (accel, gyro, quat, stale) for the dashboard's ZED IMU
        diagnostic display. `stale` is True whenever the ZED SDK's own
        per-sample IMU timestamp hasn't advanced for >1s — see the
        __init__ comment on _last_zed_imu_ts for why this check exists
        (get_sensors_data() can return SUCCESS with a cached, non-advancing
        sample during a USB HID glitch).
        """
        accel = [0.0, 0.0, 0.0]
        gyro = [0.0, 0.0, 0.0]
        quat = [0.0, 0.0, 0.0, 1.0]
        if not zed_imu:
            return accel, gyro, quat, True
        accel = zed_imu.get("linear_acceleration", accel)
        gyro = zed_imu.get("angular_velocity", gyro)
        quat = zed_imu.get("orientation", quat)
        ts = zed_imu.get("timestamp", 0)
        now_mono = time.monotonic()
        if ts and ts != self._last_zed_imu_ts:
            self._last_zed_imu_ts = ts
            self._last_zed_imu_change_time = now_mono
        stale = now_mono - self._last_zed_imu_change_time > 1.0
        return accel, gyro, quat, stale

    def run(self):
        camera_warned = False
        while self._running:
            # If the camera never opened (or dropped — e.g. the USB
            # re-enumeration glitch this device shows on open), grab()
            # itself would still be called in this loop's near-zero-delay
            # retry, and the ZED SDK logs "sl::Camera::Open has not been
            # called" on every single call — thousands of lines per second,
            # which is what actually made this look like "the system broke
            # completely." Back off hard instead of hammering a dead SDK.
            if not self.camera.is_opened():
                if not camera_warned:
                    print("[AcquisitionThread] Camera is not opened — "
                          "pausing grab() until it reconnects.")
                    camera_warned = True
                time.sleep(0.5)
                continue
            camera_warned = False

            cpu_grab = time.monotonic_ns()
            frame = self.camera.grab()
            if frame is None:
                time.sleep(0.001)
                continue

            self._frame_count += 1
            elapsed = time.monotonic() - self._t0
            fps = self._frame_count / elapsed if elapsed > 0 else 0

            # BGR → RGB in acquisition thread (not GUI)
            left_rgb = cv2.cvtColor(frame.left, cv2.COLOR_BGR2RGB)
            right_rgb = cv2.cvtColor(frame.right, cv2.COLOR_BGR2RGB)

            # Sync
            cpu_sync = time.monotonic_ns()
            synced = self.sync.sync(frame)
            if synced:
                synced.frame_uuid = str(uuid.uuid4())
            sync_err = synced.delta_ns / 1e6 if synced else 999.0

            # IMU data — preserve last valid MTi values across gaps
            mti_stale = False
            mti_state = ""
            if synced and synced.imu:
                imu = synced.imu
                a = imu.acceleration
                g = imu.gyroscope
                q = imu.quaternion
                if a:
                    self._last_mti_accel = [a[0], a[1], a[2]]
                if g:
                    self._last_mti_gyro = [g[0], g[1], g[2]]
                if q and len(q) >= 4:
                    self._last_mti_quat = [q[0], q[1], q[2], q[3]]
                self._imu_count += 1
            elif synced and not synced.imu:
                mti_stale = True
                mti_state = "NO_PACKET"
            else:
                mti_stale = True
                mti_state = "NO_SYNC"
            accel = list(self._last_mti_accel)
            gyro = list(self._last_mti_gyro)
            quat = list(self._last_mti_quat)

            # ZED IMU — diagnostic only, MTi-8 is sole navigation sensor.
            # Always captured for the dashboard; the dataset writer gates
            # ZED IMU persistence behind ENABLE_ZED_IMU_LOGGING separately.
            zed_accel, zed_gyro, zed_quat, zed_imu_stale = (
                self._read_zed_imu(frame.zed_imu)
            )

            # Radar data
            radar_stale = True
            radar_points = []
            radar_range_doppler = None
            radar_range_azimuth = None
            radar_range_profile = None
            radar_ts_ns = 0
            radar_frame_id = 0
            radar_num_objects = 0
            # A matched radar frame (even with zero detections) proves the data
            # port is connected and streaming, so use frame presence, not point
            # count, to gate connectivity.
            if synced and synced.radar and synced.radar.timestamp_ns:
                radar_points = synced.radar.points
                radar_range_doppler = synced.radar.range_doppler
                radar_range_azimuth = synced.radar.range_azimuth
                radar_range_profile = synced.radar.range_profile
                radar_ts_ns = synced.radar.timestamp_ns
                radar_frame_id = synced.radar.frame_id
                radar_num_objects = synced.radar.num_objects
                radar_stale = False

            # Live lidar is independent of camera/IMU synchronization.  The
            # receiver owns its ingest thread and atomically swaps the newest
            # immutable frame, so this never blocks UDP reception or Qt.
            #
            # The matched frame below remains exclusively for recording.
            live_lidar = self.sync.latest_lidar_frame()
            lidar_points = None
            lidar_ts_ns = 0
            lidar_frame_id = 0
            lidar_num_points = 0
            if live_lidar is not None:
                lidar_points = live_lidar.points_full
                lidar_ts_ns = live_lidar.timestamp_ns
                lidar_frame_id = live_lidar.frame_id
                lidar_num_points = live_lidar.num_points

            # Dataset synchronization remains strict: it uses only the
            # nearest scan selected by Synchronizer after the camera↔IMU gate.
            lidar_matched = False
            lidar_sync_delta_ms = None
            if synced and synced.lidar and synced.lidar.timestamp_ns:
                lidar_matched = True
                lidar_sync_delta_ms = abs(
                    synced.lidar.timestamp_ns - frame.timestamp_ns
                ) / 1e6

            # IMU rate
            now = time.monotonic()
            imu_dt = now - self._imu_last_time if self._imu_last_time else 1.0
            imu_rate = 0.0
            if imu_dt >= 1.0:
                imu_rate = (
                    (self._imu_count - self._imu_last_count) / imu_dt
                )
                self._imu_last_count = self._imu_count
                self._imu_last_time = now

            # Writer — the camera frame is always recorded when self._recording
            # is set, regardless of whether `synced` matched any other sensor.
            # `synced` is passed only for informational preview columns; MTi,
            # radar, and GNSS are persisted independently by their own
            # structured pollers (see run_gcs.py on_start), not from here.
            cpu_write = time.monotonic_ns()
            if self._recording:
                self.writer.write_camera_frame(frame, synced)
            wq = self.writer.write_queue_size
            bw = self.writer.bytes_written
            dr = self.writer.dropped
            self._bytes_written = bw

            # Full-frame reductions are deliberately sampled, not calculated
            # at camera rate.  They are dashboard telemetry, not capture data.
            if now - self._last_metrics_time >= DASHBOARD_METRICS_INTERVAL_S:
                valid_depth_pct = 0.0
                invalid_depth_pct = 0.0
                mean_confidence = 0.0
                median_depth = 0.0
                if frame.depth is not None and frame.depth.size > 0:
                    d = frame.depth
                    valid_mask = np.isfinite(d) & (d > 0)
                    total = d.size
                    n_valid = int(valid_mask.sum())
                    valid_depth_pct = 100.0 * n_valid / total if total else 0.0
                    invalid_depth_pct = 100.0 - valid_depth_pct
                    if n_valid > 0:
                        median_depth = float(np.median(d[valid_mask]))
                if frame.confidence is not None and frame.confidence.size > 0:
                    c = frame.confidence
                    valid_c = c[np.isfinite(c)]
                    if valid_c.size > 0:
                        mean_confidence = float(np.mean(valid_c))
                self._depth_metrics = (
                    valid_depth_pct, invalid_depth_pct,
                    mean_confidence, median_depth,
                )
                self._last_metrics_time = now
            (valid_depth_pct, invalid_depth_pct,
             mean_confidence, median_depth) = self._depth_metrics

            # Disk free
            try:
                df = shutil.disk_usage("/").free / (1024 ** 3)
            except Exception:
                df = 0.0

            # Recording duration
            dur = 0.0
            if self._recording:
                dur = time.time() - self.buffer.recording_start

            # Update shared buffer
            new = SharedFrame()
            new.left_rgb = left_rgb
            new.right_rgb = right_rgb
            new.depth = frame.depth
            new.confidence = frame.confidence
            new.disparity = frame.disparity
            new.view_depth = frame.view_depth
            new.view_confidence = frame.view_confidence
            new.timestamp_ns = frame.timestamp_ns
            new.frame_id = frame.frame_id
            new.frame_uuid = str(uuid.uuid4())
            new.camera_fps = fps
            new.accel = accel
            new.gyro = gyro
            new.quat = quat
            new.zed_accel = zed_accel
            new.zed_gyro = zed_gyro
            new.zed_quat = zed_quat
            new.zed_imu_stale = zed_imu_stale
            new.sync_error_ms = sync_err
            new.frame_count = self._frame_count
            new.dropped_frames = dr
            new.bytes_written = bw
            new.imu_rate = imu_rate
            new.imu_packets = self._imu_count
            new.recording = self._recording
            new.recording_duration_s = dur
            new.write_queue_size = wq
            new.write_queue_peak = max(
                getattr(self.buffer, "write_queue_peak", 0), wq
            )
            new.disk_free_gb = df
            new.exposure = frame.exposure
            new.gain = frame.gain
            new.camera_temp = frame.temperature
            new.cpu_grab_ns = cpu_grab
            new.cpu_sync_ns = cpu_sync
            new.cpu_write_ns = cpu_write
            new.valid_depth_pct = valid_depth_pct
            new.invalid_depth_pct = invalid_depth_pct
            new.mean_confidence = mean_confidence
            new.median_depth = median_depth
            new.hdf5_queue_depth = getattr(self.writer, 'sensor_writer_queue_peak', 0)
            new.sensor_queue_peak = max(
                getattr(self.buffer, "sensor_queue_peak", 0),
                getattr(self.writer, 'sensor_writer_queue_peak', 0),
            )
            new.mti_samples_written = getattr(self.writer, 'mti_samples_written', 0)
            new.mti_drops = getattr(self.writer, 'mti_drops', 0)
            new.mti_stale = mti_stale
            new.mti_state = mti_state
            new.session_id = getattr(self.writer, 'session_id', '')
            new.dataset_root = str(getattr(self.writer, 'root', ''))
            g = synced.gnss if synced else None
            new.lat = g.latitude if g else 0.0
            new.lon = g.longitude if g else 0.0
            new.alt = g.altitude if g else 0.0
            new.gnss_fix = g.fix_quality if g else 0
            new.gnss_satellites = g.satellites if g else 0
            new.gnss_hdop = g.hdop if g else 0.0
            new.gnss_speed = g.speed_mps if g else 0.0
            new.gnss_course = g.course_deg if g else 0.0
            new.rtk_fixed = g.rtk_fixed if g else False
            new.rtk_float = (g.fix_quality == 5) if g else False
            new.ntrip_connected = self.sync.ntrip_connected
            new.recording_start = self.buffer.recording_start
            new.radar_points = radar_points
            new.radar_range_doppler = radar_range_doppler
            new.radar_range_azimuth = radar_range_azimuth
            new.radar_ts_ns = radar_ts_ns
            new.radar_frame_id = radar_frame_id
            new.radar_num_objects = radar_num_objects
            new.radar_stale = radar_stale
            new.radar_range_profile = radar_range_profile
            new.radar_streaming = self.sync.radar_streaming
            new.radar_rate = self.sync.radar_rate
            new.lidar_points = lidar_points
            new.lidar_ts_ns = lidar_ts_ns
            new.lidar_frame_id = lidar_frame_id
            new.lidar_num_points = lidar_num_points
            new.lidar_streaming = self.sync.lidar_streaming
            new.lidar_rate = self.sync.lidar_rate
            new.lidar_matched = lidar_matched
            new.lidar_sync_delta_ms = lidar_sync_delta_ms
            self.buffer.update_from(new)
