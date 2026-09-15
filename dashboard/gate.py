"""Pre-recording health gate.

Run all checks before allowing recording to start.
Returns (ok, list_of_issues) where each issue is a (name, message, severity) tuple.
"""
from __future__ import annotations
import os
import shutil
from dataclasses import dataclass
from typing import Optional

from config.settings import SAVE_SVO, SAVE_RAW_IMU


@dataclass
class HealthCheck:
    name: str
    ok: bool
    message: str
    critical: bool = True  # critical = blocks recording


def check_camera(camera) -> HealthCheck:
    if not camera or not camera.is_opened():
        return HealthCheck("Camera", False, "ZED 2 not connected", True)
    return HealthCheck("Camera", True, "Connected")


def check_imu(mti) -> HealthCheck:
    if not mti or not mti.running:
        return HealthCheck("IMU", False, "MTi-8 not connected", True)
    return HealthCheck("IMU", True, f"Connected — {mti.rate:.0f} Hz")


def check_gnss(gnss) -> HealthCheck:
    if gnss is None:
        return HealthCheck("GNSS", True, "Not configured (optional)", False)
    if not gnss.running:
        return HealthCheck("GNSS", False, "GNSS not connected", False)
    if gnss.ntrip_connected:
        return HealthCheck("GNSS", True, "Connected — NTRIP RTK")
    return HealthCheck("GNSS", True, "Connected — serial (no RTK)", False)


def check_disk_free(writer) -> HealthCheck:
    try:
        usage = shutil.disk_usage(str(writer.root))
        free_gb = usage.free / (1024**3)
        # With full recording (SVO + LiDAR + Radar + IMU: ~400 MB/min),
        # require 1.5 GB minimum (supports ~3.5 minutes of continuous recording)
        if free_gb < 1.5:
            return HealthCheck("Disk", False, f"Only {free_gb:.1f} GB free (< 1.5 GB)", True)
        if free_gb < 10:
            return HealthCheck("Disk", True, f"{free_gb:.1f} GB free (warning: low)", False)
        return HealthCheck("Disk", True, f"{free_gb:.1f} GB free")
    except Exception as e:
        return HealthCheck("Disk", False, f"Cannot check disk: {e}", True)


def check_calibration(writer) -> HealthCheck:
    from pathlib import Path
    calib_path = Path(writer.root) / "calibration" / "zed_intrinsics.yaml"
    if not calib_path.exists():
        return HealthCheck("Calibration", False, "No calibration files", True)
    return HealthCheck("Calibration", True, "Present")


def check_sync(sync) -> HealthCheck:
    # Synchronization is a derived/live-preview feature only — it does not
    # gate recording. Each sensor is recorded independently regardless of
    # whether camera<->IMU (or any other pair) currently matches.
    if not sync:
        return HealthCheck(
            "Sync", False,
            "Synchronizer not initialized (preview/derived data only — "
            "recording is unaffected)",
            False,
        )
    return HealthCheck("Sync", True, "Ready (preview/derived data only)", False)


def check_radar(radar) -> HealthCheck:
    if radar is None:
        return HealthCheck("Radar", True, "Not configured (optional)", False)
    if not getattr(radar, "streaming", False):
        return HealthCheck("Radar", False, "Radar not streaming", False)
    return HealthCheck("Radar", True, f"Streaming — {radar.rate:.1f} Hz", False)


def check_lidar(lidar) -> HealthCheck:
    if lidar is None:
        return HealthCheck("Lidar", True, "Not configured (optional)", False)
    if not getattr(lidar, "streaming", False):
        return HealthCheck("Lidar", False, "Lidar not streaming", False)
    return HealthCheck("Lidar", True, f"Streaming — {lidar.rate:.1f} Hz", False)


def check_settings() -> HealthCheck:
    from config.settings import ENABLE_ZED_IMU_LOGGING
    issues = []
    if SAVE_SVO:
        issues.append("SVO")
    if SAVE_RAW_IMU:
        issues.append("Raw IMU")
    if ENABLE_ZED_IMU_LOGGING:
        issues.append("ZED IMU diagnostic")
    if not issues:
        return HealthCheck("Settings", False, "No recording outputs enabled", True)
    return HealthCheck("Settings", True, f"Recording: {', '.join(issues)} | Nav: MTi-8 sole source")


def run_health_check(camera, mti, gnss, writer, sync,
                     radar=None, lidar=None) -> tuple[bool, list[HealthCheck]]:
    """Run all checks. Returns (all_ok, checks_list).

    Sync/radar/lidar checks are informational only — recording of any given
    sensor depends solely on that sensor's own connectivity, never on
    another sensor being available or in sync.
    """
    checks = [
        check_camera(camera),
        check_imu(mti),
        check_gnss(gnss),
        check_radar(radar),
        check_lidar(lidar),
        check_disk_free(writer),
        check_calibration(writer),
        check_sync(sync),
        check_settings(),
    ]
    all_ok = all(c.ok for c in checks if c.critical)
    return all_ok, checks
