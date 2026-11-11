"""Immutable per-session configuration snapshot.

Written once at recording start (DatasetWriter.save_config_snapshot) so a
session's effective configuration can be audited later without relying on
whatever config/settings.py happens to contain today. Never includes NTRIP
credentials or any other secret.
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

from config import settings as S

_REPO_ROOT = Path(__file__).resolve().parent.parent


def _git_commit() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=str(_REPO_ROOT),
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip()
    except Exception:
        return ""


def _package_version() -> str:
    try:
        import tomllib
    except ImportError:
        try:
            import tomli as tomllib  # type: ignore
        except ImportError:
            return ""
    try:
        with open(_REPO_ROOT / "pyproject.toml", "rb") as f:
            data = tomllib.load(f)
        return data.get("project", {}).get("version", "")
    except Exception:
        return ""


def _host_clock_info() -> dict:
    info = {
        "clock_domain": (
            "Every sensor timestamps its own frames with host "
            "CLOCK_MONOTONIC (time.monotonic_ns) at the moment of "
            "reception/decode — this is the in-memory/live value used for "
            "nearest-match lookups. Before anything is persisted to disk "
            "(HDF5 timestamp_ns/host_timestamp_ns, Parquet timestamp_ns in "
            "metadata/frames, metadata/radar, metadata/gnss, and filename "
            "date prefixes), each recorder converts it to a UTC epoch "
            "nanosecond value via a mono->wall offset "
            "(time.time_ns() - time.monotonic_ns(), captured once when "
            "that recorder starts — see dataset/clock.py). Persisted "
            "timestamp_ns fields are therefore UTC epoch, NOT raw "
            "monotonic, despite sharing the field name with the live "
            "in-memory value. wall_timestamp_ns (camera only) is the same "
            "kind of conversion, captured directly at grab() time instead."
        ),
        "ptp_implemented": False,
        "gps_pps_implemented": False,
        "ntp_assumed": True,
        "ntp_verified": False,
    }
    try:
        info["monotonic_resolution_s"] = time.clock_getres(time.CLOCK_MONOTONIC)
    except Exception:
        info["monotonic_resolution_s"] = None
    return info


def _ntrip_public_config() -> dict:
    """Host/port/mountpoint only — never username/password."""
    try:
        from config.ntrip_credentials import NTRIP_CASTERS, DEFAULT_MOUNTPOINT
        casters = [
            {"name": c.get("name", ""), "host": c.get("host", ""),
             "port": c.get("port", 0)}
            for c in NTRIP_CASTERS
        ]
        return {"casters": casters, "mountpoint": DEFAULT_MOUNTPOINT}
    except Exception:
        return {"casters": [], "mountpoint": ""}


def build_config_snapshot(camera_calib: dict | None, mti, radar, lidar, gnss,
                           session_id: str) -> dict:
    return {
        "session_id": session_id,
        "recording_start_utc": time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
        ),
        "software_version": _package_version(),
        "git_commit": _git_commit(),
        "host_clock": _host_clock_info(),
        "camera": {
            "resolution": str(S.CAMERA_RESOLUTION),
            "fps": S.CAMERA_FPS,
            "depth_mode": str(S.DEPTH_MODE),
            "depth_min_m": S.DEPTH_MIN_M,
            "depth_max_m": S.DEPTH_MAX_M,
            "svo_compression_mode": getattr(
                S, "SVO_COMPRESSION_MODE", "H264_LOSSLESS"
            ),
            "effective": camera_calib or {},
        },
        "mti": {
            "port": S.MTI_PORT,
            "baudrate": S.MTI_BAUDRATE,
            "connected": bool(getattr(mti, "running", False)),
            "firmware": getattr(mti, "firmware", ""),
        },
        "radar": {
            "cfg_port": S.RADAR_CFG_PORT,
            "data_port": S.RADAR_DATA_PORT,
            "num_range_bins": S.RADAR_NUM_RANGE_BINS,
            "num_doppler_bins": S.RADAR_NUM_DOPPLER_BINS,
            "enabled": radar is not None,
            "known_limitation": (
                "Only TLV1 (detected points) and TLV5 (range-Doppler "
                "heatmap) are enabled in the sensor config; raw ADC/IQ is "
                "not captured — this driver only consumes the TI mmWave "
                "SDK's post-processed TLV UART output, not LVDS/DCA1000."
            ),
        },
        "lidar": {
            "hostname": S.LIDAR_HOSTNAME,
            "timestamp_mode_requested": S.LIDAR_TIMESTAMP_MODE,
            "timestamp_mode_active": getattr(lidar, "timestamp_mode", None),
            "enabled": lidar is not None,
        },
        "gnss": {
            "enabled": gnss is not None,
            "ntrip": _ntrip_public_config(),
        },
        "synchronization": {
            "camera_imu_tolerance_ns": S.MAX_DELTA_NS,
            "camera_radar_tolerance_ns": S.RADAR_MAX_DELTA_NS,
            "camera_lidar_tolerance_ns": S.LIDAR_MAX_DELTA_NS,
            "note": (
                "Synchronization is a derived/offline operation (see "
                "fusion/offline_sync.py) and does not gate recording."
            ),
        },
        "calibration_version": S.CALIBRATION_VERSION,
    }
