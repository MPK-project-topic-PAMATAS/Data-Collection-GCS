"""Offline synchronization — a derived operation, never a recording gate.

Every recording session lives in its own self-contained subfolder,
<dataset_root>/<session_id>/ (see DatasetWriter.begin_session() and
dataset/reader.py's module docstring). This reads the independent
native-rate per-sensor streams one such session folder already holds
(metadata/frames, sensors/*.h5, metadata/radar, metadata/gnss, and that
session's own raw/lidar.pcap + config JSON) and, for each camera frame,
finds the nearest sample in every other modality plus the residual (dt).
Writes the result to that same session's metadata/synchronization/ as a
derived index — it never modifies, deletes, or duplicates the source data;
every row is just a reference (timestamp + index) back to the native
stream it came from.

Lidar timestamps come directly from the pcap's own packet-capture
timestamps (via dataset.reader.iter_lidar_frames) — there is no per-frame
lidar JSON to read, by design.

Pass session_id to pick a specific session under the dataset root; without
one, the most recently modified session (via dataset.reader.list_sessions)
is used. Because each session is now a fully isolated folder, no
session_id column filtering is needed — everything read from that
session's own subtree already belongs to it. MTi HDF5 rotates by session
like everything else here now, so there is no cross-session bleed to worry
about.

This intentionally re-implements nearest-neighbour matching rather than
reusing Synchronizer/MTiReceiver.get_nearest(): those operate on live,
bounded, in-memory deques for the dashboard, while this operates on the
complete recorded arrays after the fact.
"""

from __future__ import annotations

import glob
from pathlib import Path

import h5py
import numpy as np
import pandas as pd

from config.settings import (
    MAX_DELTA_NS, RADAR_MAX_DELTA_NS, LIDAR_MAX_DELTA_NS, GNSS_MAX_DELTA_NS,
)
from dataset.parquet_index import SyncIndexWriter
from dataset.reader import iter_lidar_frames, resolve_lidar_paths, list_sessions


def _read_partitioned_parquet(dirpath: Path) -> pd.DataFrame:
    files = sorted(glob.glob(str(dirpath / "**" / "*.parquet"), recursive=True))
    if not files:
        return pd.DataFrame()
    return pd.concat((pd.read_parquet(f) for f in files), ignore_index=True)


def _read_mti_timestamps(sensors_dir: Path) -> np.ndarray:
    timestamps = []
    for h5_path in sorted(sensors_dir.glob("*.h5")):
        try:
            with h5py.File(str(h5_path), "r") as f:
                if "mti" in f and "timestamp_ns" in f["mti"]:
                    timestamps.append(np.asarray(f["mti"]["timestamp_ns"][:]))
        except Exception:
            continue
    if not timestamps:
        return np.array([], dtype=np.int64)
    return np.sort(np.concatenate(timestamps))


def _read_lidar_timestamps(dataset_root: Path, session_id: str | None) -> np.ndarray:
    """Decode one session's pcap + config JSON and return every scan's
    packet-capture timestamp, sorted. Empty if no lidar was recorded.
    """
    pcap_path, meta_path = resolve_lidar_paths(dataset_root, session_id)
    if pcap_path is None:
        return np.array([], dtype=np.int64)
    timestamps = []
    try:
        for frame in iter_lidar_frames(pcap_path, meta_path):
            timestamps.append(frame.timestamp_ns)
    except Exception:
        return np.array([], dtype=np.int64)
    return np.sort(np.array(timestamps, dtype=np.int64))


def _nearest_match(sorted_ts: np.ndarray, target: int) -> tuple[int, int]:
    """Return (matched_timestamp_ns, dt_ns) for the entry in sorted_ts
    nearest to target, or (0, -1) if sorted_ts is empty."""
    if sorted_ts.size == 0:
        return 0, -1
    idx = np.searchsorted(sorted_ts, target)
    candidates = []
    if idx < sorted_ts.size:
        candidates.append(int(sorted_ts[idx]))
    if idx > 0:
        candidates.append(int(sorted_ts[idx - 1]))
    best = min(candidates, key=lambda t: abs(t - target))
    return best, abs(best - target)


def synchronize_session(
    root: str | Path,
    session_id: str | None = None,
    tau_imu_ns: int = MAX_DELTA_NS,
    tau_radar_ns: int = RADAR_MAX_DELTA_NS,
    tau_lidar_ns: int = LIDAR_MAX_DELTA_NS,
    tau_gnss_ns: int = GNSS_MAX_DELTA_NS,
) -> int:
    """Build <session>/metadata/synchronization/ from one session's native
    streams.

    session_id picks which session under the dataset root to process; omit
    it to use the most recently modified session (dataset.reader.
    list_sessions). Every stream read here — frames, MTi HDF5, radar, gnss,
    lidar pcap — comes from that one session's own self-contained
    subfolder, so no session_id column filtering is needed.

    Returns the number of rows written. Safe to re-run — later runs append
    additional Parquet parts; source data is never modified.
    """
    dataset_root = Path(root)
    if session_id is None:
        sessions = list_sessions(dataset_root)
        if not sessions:
            return 0
        session_id = sessions[0]
    session_root = dataset_root / session_id
    meta_dir = session_root / "metadata"

    frames = _read_partitioned_parquet(meta_dir / "frames")
    if frames.empty:
        return 0
    frames = frames.sort_values("timestamp_ns")

    mti_ts = _read_mti_timestamps(session_root / "sensors")
    radar_df = _read_partitioned_parquet(meta_dir / "radar")
    radar_ts = (
        np.sort(radar_df["timestamp_ns"].to_numpy())
        if not radar_df.empty else np.array([], dtype=np.int64)
    )
    gnss_df = _read_partitioned_parquet(meta_dir / "gnss")
    gnss_ts = (
        np.sort(gnss_df["timestamp_ns"].to_numpy())
        if not gnss_df.empty else np.array([], dtype=np.int64)
    )
    lidar_ts = _read_lidar_timestamps(dataset_root, session_id)

    writer = SyncIndexWriter(metadata_dir=meta_dir, flush_interval=1000)
    n = 0
    for _, row in frames.iterrows():
        cam_ts = int(row["timestamp_ns"])
        imu_match, dt_imu = _nearest_match(mti_ts, cam_ts)
        radar_match, dt_radar = _nearest_match(radar_ts, cam_ts)
        lidar_match, dt_lidar = _nearest_match(lidar_ts, cam_ts)
        gnss_match, dt_gnss = _nearest_match(gnss_ts, cam_ts)

        writer.add_row({
            "camera_timestamp_ns": cam_ts,
            "camera_frame_id": int(row["frame_id"]),
            "imu_timestamp_ns": imu_match,
            "dt_camera_imu_ns": dt_imu,
            "imu_in_tolerance": 0 <= dt_imu <= tau_imu_ns,
            "radar_timestamp_ns": radar_match,
            "dt_camera_radar_ns": dt_radar,
            "radar_in_tolerance": 0 <= dt_radar <= tau_radar_ns,
            "lidar_timestamp_ns": lidar_match,
            "dt_camera_lidar_ns": dt_lidar,
            "lidar_in_tolerance": 0 <= dt_lidar <= tau_lidar_ns,
            "gnss_timestamp_ns": gnss_match,
            "dt_camera_gnss_ns": dt_gnss,
            "gnss_in_tolerance": 0 <= dt_gnss <= tau_gnss_ns,
        })
        n += 1
    writer.flush()
    return n
