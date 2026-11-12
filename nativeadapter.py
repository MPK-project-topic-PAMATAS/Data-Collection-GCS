"""Native acquisition facades — drop-in replacements for the per-sensor
Python adapters (ZEDCamera, MTiReceiver, GNSSReceiver, RadarReceiver,
LidarReceiver) backed by the C++ ``AcquisitionManager``.

Each facade mirrors the *public interface* of its Python counterpart and returns
the *same dataclass types* (``dataset.Frame``, ``mti8.MTiFrame``,
``radar.RadarFrame``, ``lidar.LidarFrame``, GNSS ``dict``). This keeps
``Synchronizer``, ``AcquisitionThread``, ``DatasetWriter`` and the recording
hooks in ``run_gcs.py`` completely unchanged.

Designed to only be used when ``sensor_backend.available()`` is True; the app
falls back to the original Python adapters otherwise. No sensors run merely by
importing this module — ``NativeAcquisition.start()`` must be called.
"""

from __future__ import annotations

import time

import numpy as np

import sensor_backend as _sb

# Lazy imports: these trigger transitive deps (pyzed, serial, etc.) that may
# not be present in test environments.  At runtime in run_gcs.py they are
# guaranteed available.
_Frame = None
_PyMTiFrame = None
_PyRadarFrame = None
_PyRadarPoint = None
_PyLidarFrame = None


def _ensure_frame():
    global _Frame, _PyMTiFrame, _PyRadarFrame, _PyRadarPoint, _PyLidarFrame
    if _Frame is None:
        from dataset.frame import Frame as _Frame
    if _PyMTiFrame is None:
        from mti8.frame import MTiFrame as _PyMTiFrame
    if _PyRadarFrame is None:
        from radar.frame import RadarFrame as _PyRadarFrame
    if _PyRadarPoint is None:
        from radar.frame import RadarPoint as _PyRadarPoint
    if _PyLidarFrame is None:
        from lidar.frame import LidarFrame as _PyLidarFrame


# ── frame conversion helpers (native -> existing Python dataclasses) ──────

def _to_mti_frame(nf) -> "PyMTiFrame":
    """Native MTiFrame -> mti8.MTiFrame."""
    _ensure_frame()
    return _PyMTiFrame(
        packet_counter=nf.packet_counter,
        sample_time_fine=nf.sample_time_fine,
        sample_time_coarse=nf.sample_time_coarse,
        quaternion=tuple(nf.quaternion),          # (w, x, y, z) both sides
        euler=tuple(nf.euler) if nf.euler else None,
        acceleration=tuple(nf.acceleration),
        gyroscope=tuple(nf.gyroscope),
        magnetometer=(tuple(nf.magnetometer) if nf.magnetometer else None),
        velocity=(tuple(nf.velocity) if nf.velocity else None),
        latitude=nf.latitude,
        longitude=nf.longitude,
        altitude_msl=nf.altitude_msl,
        altitude_ellipsoid=nf.altitude_ellipsoid,
        temperature=nf.temperature,
        baro_pressure=nf.baro_pressure,
        status_word=nf.status_word,
        status_byte=nf.status_byte,
        timestamp_ns=nf.timestamp_ns,
    )


def _to_radar_frame(nf):
    """Native RadarFrame -> radar.RadarFrame.

    Returns None when the frame carries no data (e.g. radar offline) so callers
    gracefully fall back to an empty RadarData(), matching the Python receiver
    which returns None from get_nearest when its buffer is empty.
    """
    if nf.range_doppler is None:
        return None
    # Accept any array-like (the binding yields numpy arrays; synthetic frames
    # may use lists), mirroring _to_lidar_frame.
    range_doppler = np.asarray(nf.range_doppler, dtype=np.uint16)
    if range_doppler.size == 0:
        return None
    _ensure_frame()
    points = [_PyRadarPoint(x=p.x, y=p.y, z=p.z, velocity_mps=p.velocity_mps)
              for p in nf.points]
    range_doppler = range_doppler.reshape(nf.num_range_bins, nf.num_doppler_bins)
    range_profile = np.asarray(nf.range_profile, dtype=np.float32)
    return _PyRadarFrame(
        points=points,
        range_doppler=range_doppler,
        range_profile=range_profile,
        timestamp_ns=nf.timestamp_ns,
        frame_id=nf.frame_id,
        num_objects=nf.num_objects,
        num_range_bins=nf.num_range_bins,
        num_doppler_bins=nf.num_doppler_bins,
    )


def _to_lidar_frame(nf):
    """Native OusterFrame -> lidar.LidarFrame (points == points_full)."""
    _ensure_frame()
    points = np.asarray(nf.points).reshape(-1, 3).copy() if nf.num_points else None
    refl = np.asarray(nf.reflectivity).copy() if nf.reflectivity is not None else None
    return _PyLidarFrame(
        frame_id=nf.frame_id,
        timestamp_ns=nf.timestamp_ns,
        points=points,
        points_full=points,
        reflectivity=refl,
        num_points=nf.num_points,
        complete=nf.complete,
    )


def _to_gnss_dict(gf) -> dict:
    if gf is None:
        return None
    return {
        "lat": gf.latitude,
        "lon": gf.longitude,
        "alt": gf.altitude,
        "fix": "RTK Fixed" if gf.rtk_fixed else ("3D Fix" if gf.has_fix else "No Fix"),
        "fix_quality": gf.fix_quality,
        "satellites": gf.satellites,
        "hdop": gf.hdop,
        "speed": gf.speed_mps,
        "course": gf.course_deg,
        "rtk_fixed": gf.rtk_fixed,
        "utc_time": "",
        "gps_week": 0,
        "gps_tow": 0.0,
        "timestamp_ns": gf.timestamp_ns,
    }


# ── facades ───────────────────────────────────────────────────────────────

class NativeZEDCamera:
    """Drop-in for camera.ZEDCamera, reading native ZedFrames from the manager.

    ``grab()`` returns a ``dataset.Frame`` with BGR images (matching the Python
    adapter so ``AcquisitionThread``'s BGR->RGB conversion is unchanged).
    """

    def __init__(self, manager):
        self._mgr = manager
        self.frame_id = 0
        self.calib = None
        self._recording = False
        self._last_frame_id = -1

    def is_opened(self) -> bool:
        return self._mgr.zed_opened()

    def open(self):
        if not self._mgr.zed_opened():
            raise RuntimeError("Failed to open ZED camera via native backend")
        w, h = self._mgr.zed_width(), self._mgr.zed_height()
        self.calib = {
            "model": "ZED 2", "serial": "native",
            "sdk_version": "native-cpp",
            "fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0,
            "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
            "baseline": 0.0, "width": w, "height": h, "fps": 0,
        }

    def grab(self) -> Frame | None:
        nf = self._mgr.zed_latest()
        if nf is None or nf.frame_id <= self._last_frame_id:
            return None
        self._last_frame_id = nf.frame_id
        self.frame_id = nf.frame_id

        import cv2
        left_bgr = cv2.cvtColor(np.asarray(nf.left), cv2.COLOR_RGB2BGR)
        right_bgr = cv2.cvtColor(np.asarray(nf.right), cv2.COLOR_RGB2BGR)
        depth = np.asarray(nf.depth).copy()
        confidence = np.asarray(nf.confidence).copy() if nf.confidence is not None else None
        disparity = np.asarray(nf.disparity).copy() if nf.disparity is not None else None

        zed_imu = {
            "linear_acceleration": list(nf.zed_accel),
            "angular_velocity": list(nf.zed_gyro),
            "orientation": list(nf.zed_quat),          # (x, y, z, w)
            "magnetic_field": None,
            "temperature": nf.zed_imu_temperature,
            "timestamp": nf.zed_imu_timestamp_ns,
        }

        _ensure_frame()
        frame = _Frame(
            frame_id=nf.frame_id,
            timestamp_ns=nf.timestamp_ns,
            left=left_bgr,
            right=right_bgr,
            depth=depth,
            zed_imu=zed_imu,
            confidence=confidence,
            disparity=disparity,
            view_depth=None,
            view_confidence=None,
            exposure=nf.exposure,
            gain=nf.gain,
            temperature=nf.temperature,
            wall_timestamp_ns=time.time_ns(),
        )
        return frame

    def start_recording(self, svo_path: str):
        self._mgr.zed_start_recording(svo_path)
        self._recording = True

    def stop_recording(self):
        self._mgr.zed_stop_recording()
        self._recording = False

    def close(self):
        try:
            self.stop_recording()
        except Exception:
            pass


class NativeMTiReceiver:
    """Drop-in for mti8.MTiReceiver, reading native MTi frames."""

    def __init__(self, manager):
        self._mgr = manager
        self._running = False
        self._firmware = "native"

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def start_raw_log(self, path: str):
        self._mgr.mti8_start_raw_log(path)

    def stop_raw_log(self):
        self._mgr.mti8_stop_raw_log()

    def get_frame(self):
        nf = self._mgr.mti8_latest()
        return _to_mti_frame(nf) if nf is not None else None

    def get_validated_frame(self):
        return self.get_frame(), None

    @property
    def sample_state(self):
        return None

    @property
    def counters(self):
        return (0, 0, 0, 0, 0)

    def get_nearest(self, target_ts):
        nf = self._mgr.mti8_nearest(target_ts)
        return _to_mti_frame(nf) if nf is not None else None

    @property
    def running(self):
        return self._running

    @property
    def rate(self):
        d = self._mgr.get_statistics().get("mti8")
        return float(d.measured_rate) if d else 0.0

    @property
    def firmware(self):
        return self._firmware

    @firmware.setter
    def firmware(self, value: str):
        self._firmware = value


class NativeGNSSReceiver:
    """Drop-in for gnss.GNSSReceiver (dict-latest API)."""

    def __init__(self, manager):
        self._mgr = manager
        self._running = False

    def start(self) -> bool:
        self._running = True
        return True

    def stop(self):
        self._running = False

    def latest(self) -> dict | None:
        gf = self._mgr.gnss_latest()
        return _to_gnss_dict(gf)

    @property
    def running(self):
        return self._running

    @property
    def is_connected(self):
        return bool(self.latest())

    @property
    def ntrip_connected(self):
        return False

    @property
    def fix_count(self):
        return 0

    @property
    def bytes_received(self):
        return 0


class NativeRadarReceiver:
    """Drop-in for radar.RadarReceiver."""

    def __init__(self, manager):
        self._mgr = manager

    def start(self):
        pass

    def stop(self):
        pass

    def get_frame(self):
        nf = self._mgr.radar_latest()
        return _to_radar_frame(nf) if nf is not None else None

    def get_nearest(self, target_ts: int):
        nf = self._mgr.radar_nearest(target_ts)
        return _to_radar_frame(nf) if nf is not None else None

    @property
    def rate(self) -> float:
        d = self._mgr.get_statistics().get("radar")
        return float(d.measured_rate) if d else 0.0

    @property
    def streaming(self) -> bool:
        return self._mgr.radar_connected()


class NativeLidarReceiver:
    """Drop-in for lidar.LidarReceiver."""

    def __init__(self, manager):
        self._mgr = manager
        self.sensor_name = "OS-0-64"
        self._pcap_path = None
        self._config_json_path = None
        self._sensor_info = None

    def start(self):
        pass

    def stop(self):
        pass

    def start_pcap(self, path: str) -> str:
        self._mgr.ouster_start_pcap(path)
        self._pcap_path = path

        # Save metadata JSON alongside PCAP
        try:
            import os
            import json
            stem = os.path.splitext(path)[0]
            md_path = f"{stem}_0.json"

            # Try to get sensor info from the C++ backend
            info_json = self._mgr.ouster_sensor_info_json()
            if info_json:
                with open(md_path, "w") as f:
                    f.write(info_json)
                self._config_json_path = md_path
        except Exception:
            pass  # metadata save failure should not interrupt recording

        return path

    def stop_pcap(self):
        self._mgr.ouster_stop_pcap()
        self._pcap_path = None

    @property
    def pcap_path(self) -> str | None:
        return self._pcap_path

    @property
    def config_json_path(self) -> str | None:
        return self._config_json_path

    def get_frame(self):
        nf = self._mgr.ouster_latest()
        return _to_lidar_frame(nf) if nf is not None else None

    def get_nearest(self, target_ts: int):
        nf = self._mgr.ouster_nearest(target_ts)
        return _to_lidar_frame(nf) if nf is not None else None

    @property
    def rate(self) -> float:
        d = self._mgr.get_statistics().get("ouster")
        return float(d.measured_rate) if d else 0.0

    @property
    def streaming(self) -> bool:
        return self._mgr.ouster_streaming()


class NativeAcquisition:
    """Builds a shared C++ AcquisitionManager plus all five facades.

    Mirrors how run_gcs.py constructs the Python sensors: construct the object,
    call ``.start()`` to bring every enabled native worker up, then pass the
    facade attributes into ``Synchronizer`` / ``AcquisitionThread`` unchanged.
    """

    def __init__(self, enable_zed=True, enable_mti8=True, enable_radar=True,
                 enable_ouster=True, enable_gnss=True):
        cfg = _sb.AcquisitionConfig()
        cfg.enable_zed = enable_zed
        cfg.enable_mti8 = enable_mti8
        cfg.enable_radar = enable_radar
        cfg.enable_ouster = enable_ouster
        cfg.enable_gnss = enable_gnss
        self.z = cfg.zed
        self.m = cfg.mti8
        self.r = cfg.radar
        self.o = cfg.ouster
        self.g = cfg.gnss
        self._apply_settings()
        self.manager = _sb.AcquisitionManager(cfg)

        self.camera = NativeZEDCamera(self.manager)
        self.mti = NativeMTiReceiver(self.manager)
        self.gnss = NativeGNSSReceiver(self.manager)
        self.radar = NativeRadarReceiver(self.manager)
        self.lidar = NativeLidarReceiver(self.manager)

    def _apply_settings(self):
        """Mirror config.settings values into the native configs (best-effort)."""
        try:
            from config import settings as S
            z = self.z
            z.resolution = int(S.CAMERA_RESOLUTION.value)
            z.fps = int(S.CAMERA_FPS)
            z.depth_mode = int(S.DEPTH_MODE.value)
            z.unit = int(S.UNIT.value)
            z.coord_system = int(S.COORDINATE_SYSTEM.value)
            z.depth_min = float(S.DEPTH_MIN_M)
            z.depth_max = float(S.DEPTH_MAX_M)
            z.depth_stabilization = int(S.DEPTH_STABILIZATION)
            z.confidence_thresh = int(S.CONFIDENCE_THRESHOLD)
            z.texture_conf_thresh = int(S.TEXTURE_CONFIDENCE_THRESHOLD)
            z.flip_180 = bool(S.CAMERA_FLIP_180)
            z.save_confidence = bool(S.SAVE_CONFIDENCE)
            z.save_disparity = bool(S.SAVE_DISPARITY)

            self.m.port = S.MTI_PORT
            self.m.baudrate = int(S.MTI_BAUDRATE)
            self.m.timeout_s = float(S.MTI_TIMEOUT)

            self.r.cfg_port = S.RADAR_CFG_PORT
            self.r.data_port = S.RADAR_DATA_PORT
            self.r.cfg_baud = int(S.RADAR_CFG_BAUD)
            self.r.data_baud = int(S.RADAR_DATA_BAUD)
            self.r.num_range_bins = int(S.RADAR_NUM_RANGE_BINS)
            self.r.num_doppler_bins = int(S.RADAR_NUM_DOPPLER_BINS)

            self.o.hostname = S.LIDAR_HOSTNAME
        except Exception:
            # Best-effort: native C++ defaults remain valid surprises-free.
            pass

    def start(self) -> bool:
        return self.manager.start()

    def stop(self):
        self.manager.stop()
