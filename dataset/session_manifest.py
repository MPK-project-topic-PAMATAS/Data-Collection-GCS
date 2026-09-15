"""Per-session recording manifest.

Distinct from manifests/dataset_manifest.json (a cumulative, dataset-wide
summary written once at DatasetWriter.close()) and from
metadata/config_snapshot_<session_id>.json (configuration only, no file
paths). This manifest answers one question: for this specific REC->STOP
session, exactly which files were produced, where, and over what time range.

Written once, at the end of a session (on_stop), once both the start and
stop timestamps and every recorder's actual output path are known.
"""

from __future__ import annotations

from pathlib import Path

from config.settings import (
    MAX_DELTA_NS, RADAR_MAX_DELTA_NS, LIDAR_MAX_DELTA_NS, GNSS_MAX_DELTA_NS,
)


def _relative(root: Path, path: str | None) -> str | None:
    if not path:
        return None
    try:
        return str(Path(path).resolve().relative_to(root.resolve()))
    except ValueError:
        return str(path)


def build_session_manifest(root: Path, session_id: str,
                           start_timestamp_ns: int, stop_timestamp_ns: int,
                           camera_svo2: str | None = None,
                           mti_raw_xbus: str | None = None,
                           ouster_pcap: str | None = None,
                           ouster_config_json: str | None = None,
                           radar_bag: str | None = None,
                           gnss_nmea_log: str | None = None) -> dict:
    """Build the session manifest dict.

    Every ``*_path``/file argument is the actual path the corresponding
    recorder used this session (or None if that sensor wasn't recorded).
    Exactly one Ouster pcap and one companion config JSON are expected per
    session — never one JSON per frame/scan/packet; nothing in this
    codebase should ever populate ``ouster_config_json`` with more than one
    path.
    """
    root = Path(root)
    return {
        "session_id": session_id,
        "start_timestamp_ns": start_timestamp_ns,
        "stop_timestamp_ns": stop_timestamp_ns,
        "duration_s": max(0.0, (stop_timestamp_ns - start_timestamp_ns) / 1e9),
        "files": {
            "camera_svo2": _relative(root, camera_svo2),
            "mti_raw_xbus": _relative(root, mti_raw_xbus),
            "ouster_pcap": _relative(root, ouster_pcap),
            "ouster_config_json": _relative(root, ouster_config_json),
            "radar_bag": _relative(root, radar_bag),
            "radar_native_structured": (
                "radar/points, radar/heatmap, radar/range_profile "
                "(indexed by metadata/radar/) — TLV detections + "
                "range-Doppler heatmap only; raw ADC/IQ is not captured by "
                "this hardware interface"
                if radar_bag else None
            ),
            "gnss_nmea_log": _relative(root, gnss_nmea_log),
        },
        "mti_structured": "sensors/*.h5 (indexed within the HDF5 file itself)",
        "camera_structured": "metadata/frames/ (one row per camera frame)",
        "gnss_structured": "metadata/gnss/ (one row per fix)" if gnss_nmea_log else None,
        "synchronization": {
            "applied_during_recording": False,
            "note": (
                "Every file above was recorded independently at its own "
                "native rate; synchronization never gated what was "
                "written. Build the derived, offline synchronized index "
                "with `synchronize-dataset <dataset_root>` "
                "(fusion/offline_sync.py) -> metadata/synchronization/."
            ),
            "tolerances_ns": {
                "camera_imu": MAX_DELTA_NS,
                "camera_radar": RADAR_MAX_DELTA_NS,
                "camera_lidar": LIDAR_MAX_DELTA_NS,
                "camera_gnss": GNSS_MAX_DELTA_NS,
            },
        },
    }
