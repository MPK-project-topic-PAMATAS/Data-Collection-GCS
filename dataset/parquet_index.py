from __future__ import annotations
"""Parquet frame index writer — partitioned, append-only, immutable parts.

Layout:
    metadata/frames/
        date=20260727/
            part-000001.parquet   (up to PARQUET_FLUSH_INTERVAL rows)
            part-000002.parquet
        date=20260728/
            part-000001.parquet

Schema:
    frame_id: int64
    timestamp_ns: int64
    datetime_utc: string
    session_id: string
    left_path: string
    right_path: string
    depth_path: string
    disparity_path: string
    nearest_mti_timestamp_ns: int64
    sync_delta_ns: int64
    sensor_hdf5_file: string
    nearest_mti_index: int64
    exposure: float32
    gain: float32
    temperature: float32
    brightness: float32 (optional)
    sharpness: float32 (optional)
"""

import threading
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

SCHEMA = pa.schema([
    ("frame_id", pa.int64()),
    ("timestamp_ns", pa.int64()),
    ("datetime_utc", pa.string()),
    ("session_id", pa.string()),
    ("left_path", pa.string()),
    ("right_path", pa.string()),
    ("depth_path", pa.string()),
    ("disparity_path", pa.string()),
    ("nearest_mti_timestamp_ns", pa.int64()),
    ("sync_delta_ns", pa.int64()),
    ("sensor_hdf5_file", pa.string()),
    ("nearest_mti_index", pa.int64()),
    ("exposure", pa.float32()),
    ("gain", pa.float32()),
    ("temperature", pa.float32()),
    ("radar_points_path", pa.string()),
    ("radar_heatmap_path", pa.string()),
    ("radar_range_profile_path", pa.string()),
    ("radar_timestamp_ns", pa.int64()),
    ("radar_frame_id", pa.int64()),
    ("radar_num_objects", pa.int64()),
    ("radar_sync_delta_ns", pa.int64()),
    ("radar_num_range_bins", pa.int32()),
    ("radar_num_doppler_bins", pa.int32()),
])


class _PartitionedParquetWriter:
    """Append-only Parquet writer with date-based partitioning.

    Shared by FrameIndexWriter, RadarIndexWriter, and GNSSIndexWriter — each
    only differs in its target subdirectory (under metadata/) and its
    pyarrow schema.
    """

    #: Subdirectory name under metadata_dir. Subclasses must override.
    _SUBDIR = ""
    #: pyarrow schema for rows written by this index. Subclasses must override.
    _SCHEMA: pa.Schema = None

    def __init__(self, metadata_dir: Path, flush_interval: int = 1000,
                 compression: str = "snappy"):
        self._metadata_dir = Path(metadata_dir)
        self._frames_dir = self._metadata_dir / self._SUBDIR
        self._frames_dir.mkdir(parents=True, exist_ok=True)
        self._flush_interval = flush_interval
        self._compression = compression
        self._lock = threading.Lock()
        self._buffer = []
        self._part_counter = {}  # date_str -> next part number

    def _get_part_path(self, date_str: str) -> Path:
        part_dir = self._frames_dir / f"date={date_str}"
        part_dir.mkdir(parents=True, exist_ok=True)
        counter = self._part_counter.get(date_str, 0) + 1
        self._part_counter[date_str] = counter
        return part_dir / f"part-{counter:06d}.parquet"

    def _date_str(self, timestamp_ns: int) -> str:
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        return dt.strftime("%Y%m%d")

    def _add_row(self, row: dict):
        """Buffer a pre-built row dict. Flushes when buffer is full."""
        with self._lock:
            self._buffer.append(row)
            if len(self._buffer) >= self._flush_interval:
                self._flush_locked()

    def flush(self):
        with self._lock:
            self._flush_locked()

    def _flush_locked(self):
        """Flush buffer to a new Parquet part file. Must hold _lock."""
        if not self._buffer:
            return

        # Group rows by date
        by_date = {}
        for row in self._buffer:
            date_str = self._date_str(row["timestamp_ns"])
            by_date.setdefault(date_str, []).append(row)
        self._buffer.clear()

        for date_str, rows in by_date.items():
            table = pa.Table.from_pylist(rows, schema=self._SCHEMA)
            part_path = self._get_part_path(date_str)
            pq.write_table(table, str(part_path), compression=self._compression)


class FrameIndexWriter(_PartitionedParquetWriter):
    """Append-only Parquet writer with date-based partitioning."""

    _SUBDIR = "frames"
    _SCHEMA = SCHEMA

    def add_frame(self, frame_id: int, timestamp_ns: int,
                  session_id: str, left_path: str, right_path: str,
                  depth_path: str, disparity_path: str,
                  nearest_mti_ts: int, sync_delta_ns: int,
                  sensor_hdf5_file: str, nearest_mti_index: int,
                  exposure: float = 0.0, gain: float = 0.0,
                  temperature: float = 0.0,
                  radar_points_path: str = "",
                  radar_heatmap_path: str = "",
                  radar_range_profile_path: str = "",
                  radar_timestamp_ns: int = 0,
                  radar_frame_id: int = 0,
                  radar_num_objects: int = 0,
                  radar_sync_delta_ns: int = 0,
                  radar_num_range_bins: int = 0,
                  radar_num_doppler_bins: int = 0):
        """Add a frame row to the buffer. Flushes when buffer is full."""
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        row = {
            "frame_id": frame_id,
            "timestamp_ns": timestamp_ns,
            "datetime_utc": dt.isoformat(),
            "session_id": session_id or "",
            "left_path": left_path,
            "right_path": right_path,
            "depth_path": depth_path,
            "disparity_path": disparity_path,
            "nearest_mti_timestamp_ns": nearest_mti_ts,
            "sync_delta_ns": sync_delta_ns,
            "sensor_hdf5_file": sensor_hdf5_file,
            "nearest_mti_index": nearest_mti_index,
            "exposure": float(exposure),
            "gain": float(gain),
            "temperature": float(temperature),
            "radar_points_path": radar_points_path,
            "radar_heatmap_path": radar_heatmap_path,
            "radar_range_profile_path": radar_range_profile_path,
            "radar_timestamp_ns": radar_timestamp_ns,
            "radar_frame_id": radar_frame_id,
            "radar_num_objects": radar_num_objects,
            "radar_sync_delta_ns": radar_sync_delta_ns,
            "radar_num_range_bins": radar_num_range_bins,
            "radar_num_doppler_bins": radar_num_doppler_bins,
        }
        self._add_row(row)


RADAR_SCHEMA = pa.schema([
    ("frame_id", pa.int64()),
    ("timestamp_ns", pa.int64()),
    ("datetime_utc", pa.string()),
    ("session_id", pa.string()),
    ("points_path", pa.string()),
    ("heatmap_path", pa.string()),
    ("range_profile_path", pa.string()),
    ("num_objects", pa.int64()),
    ("num_range_bins", pa.int32()),
    ("num_doppler_bins", pa.int32()),
])


class RadarIndexWriter(_PartitionedParquetWriter):
    """Native-rate radar frame index — independent of camera timing.

    One row per radar frame received, regardless of whether any camera
    frame ever matched it.
    """

    _SUBDIR = "radar"
    _SCHEMA = RADAR_SCHEMA

    def add_radar_frame(self, frame_id: int, timestamp_ns: int,
                         session_id: str,
                         points_path: str = "", heatmap_path: str = "",
                         range_profile_path: str = "",
                         num_objects: int = 0,
                         num_range_bins: int = 0,
                         num_doppler_bins: int = 0):
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        row = {
            "frame_id": frame_id,
            "timestamp_ns": timestamp_ns,
            "datetime_utc": dt.isoformat(),
            "session_id": session_id or "",
            "points_path": points_path,
            "heatmap_path": heatmap_path,
            "range_profile_path": range_profile_path,
            "num_objects": num_objects,
            "num_range_bins": num_range_bins,
            "num_doppler_bins": num_doppler_bins,
        }
        self._add_row(row)


GNSS_SCHEMA = pa.schema([
    ("timestamp_ns", pa.int64()),
    ("datetime_utc", pa.string()),
    ("session_id", pa.string()),
    ("latitude", pa.float64()),
    ("longitude", pa.float64()),
    ("altitude", pa.float64()),
    ("fix_quality", pa.int32()),
    ("fix_type", pa.string()),
    ("satellites", pa.int32()),
    ("hdop", pa.float32()),
    ("speed_mps", pa.float32()),
    ("course_deg", pa.float32()),
    ("rtk_fixed", pa.bool_()),
    ("utc_time", pa.string()),
    ("gps_week", pa.int32()),
    ("gps_tow", pa.float64()),
])


SYNC_SCHEMA = pa.schema([
    ("camera_timestamp_ns", pa.int64()),
    ("timestamp_ns", pa.int64()),  # alias of camera_timestamp_ns; partition key
    ("camera_frame_id", pa.int64()),
    ("imu_timestamp_ns", pa.int64()),
    ("dt_camera_imu_ns", pa.int64()),
    ("imu_in_tolerance", pa.bool_()),
    ("radar_timestamp_ns", pa.int64()),
    ("dt_camera_radar_ns", pa.int64()),
    ("radar_in_tolerance", pa.bool_()),
    ("lidar_timestamp_ns", pa.int64()),
    ("dt_camera_lidar_ns", pa.int64()),
    ("lidar_in_tolerance", pa.bool_()),
    ("gnss_timestamp_ns", pa.int64()),
    ("dt_camera_gnss_ns", pa.int64()),
    ("gnss_in_tolerance", pa.bool_()),
])


class SyncIndexWriter(_PartitionedParquetWriter):
    """Derived synchronized index — built offline from native per-sensor
    streams (fusion/offline_sync.py). Never written during recording, never
    a precondition for recording. References source timestamps; does not
    duplicate or discard any native sample.
    """

    _SUBDIR = "synchronization"
    _SCHEMA = SYNC_SCHEMA

    def add_row(self, row: dict):
        row = dict(row)
        row.setdefault("timestamp_ns", row["camera_timestamp_ns"])
        self._add_row(row)


class GNSSIndexWriter(_PartitionedParquetWriter):
    """Native-rate GNSS fix index — independent of camera timing.

    One row per parsed GGA/RMC fix, regardless of whether any camera frame
    ever matched it. Never includes NTRIP credentials.
    """

    _SUBDIR = "gnss"
    _SCHEMA = GNSS_SCHEMA

    def add_fix(self, timestamp_ns: int, session_id: str, fix: dict):
        dt = datetime.fromtimestamp(timestamp_ns / 1e9, tz=timezone.utc)
        row = {
            "timestamp_ns": timestamp_ns,
            "datetime_utc": dt.isoformat(),
            "session_id": session_id or "",
            "latitude": float(fix.get("lat", 0.0)),
            "longitude": float(fix.get("lon", 0.0)),
            "altitude": float(fix.get("alt", 0.0)),
            "fix_quality": int(fix.get("fix_quality", 0)),
            "fix_type": str(fix.get("fix", "")),
            "satellites": int(fix.get("satellites", 0)),
            "hdop": float(fix.get("hdop", 0.0)),
            "speed_mps": float(fix.get("speed", 0.0)),
            "course_deg": float(fix.get("course", 0.0)),
            "rtk_fixed": bool(fix.get("rtk_fixed", False)),
            "utc_time": str(fix.get("utc_time", "")),
            "gps_week": int(fix.get("gps_week", 0)),
            "gps_tow": float(fix.get("gps_tow", 0.0)),
        }
        self._add_row(row)
