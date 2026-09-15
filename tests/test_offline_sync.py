"""Offline synchronization must reconstruct nearest-neighbour matches and
residuals from independently-recorded native streams — and must never
happen during recording (it's a plain function over files on disk)."""

import sys
import glob
from types import SimpleNamespace, ModuleType

import h5py
import numpy as np
import pandas as pd

if not hasattr(sys.modules.get("pyzed"), "sl"):
    zed_module = ModuleType("pyzed")
    zed_module.sl = SimpleNamespace(
        RESOLUTION=SimpleNamespace(HD720=object()),
        DEPTH_MODE=SimpleNamespace(NEURAL_LIGHT=object()),
        UNIT=SimpleNamespace(METER=object()),
        COORDINATE_SYSTEM=SimpleNamespace(RIGHT_HANDED_Z_UP=object()),
    )
    sys.modules["pyzed"] = zed_module

from dataset.parquet_index import FrameIndexWriter, RadarIndexWriter
from fusion.offline_sync import synchronize_session, _nearest_match


def _make_session(root, session_id="s1"):
    """Lay out fixture files the way DatasetWriter.begin_session() does:
    everything under root/<session_id>/."""
    root = root / session_id
    meta = root / "metadata"

    # Camera frames every 100ms.
    fi = FrameIndexWriter(metadata_dir=meta, flush_interval=1)
    for i, ts in enumerate([1_000_000_000, 1_100_000_000, 1_200_000_000]):
        fi.add_frame(
            frame_id=i, timestamp_ns=ts, session_id="s1",
            left_path="", right_path="", depth_path="", disparity_path="",
            nearest_mti_ts=0, sync_delta_ns=-1, sensor_hdf5_file="",
            nearest_mti_index=-1,
        )
    fi.flush()

    # MTi HDF5 at ~5ms period, with one sample 60ms out-of-tolerance from
    # the middle camera frame (1_100_000_000) to test the tolerance flag.
    sensors_dir = root / "sensors"
    sensors_dir.mkdir(parents=True, exist_ok=True)
    with h5py.File(str(sensors_dir / "test_mti.h5"), "w") as f:
        g = f.create_group("mti")
        ts = np.array([1_000_002_000, 1_099_998_000, 1_200_001_000], dtype=np.int64)
        g.create_dataset("timestamp_ns", data=ts)

    # Radar every 150ms — far enough that the first camera frame (1.0s) has
    # no in-tolerance radar match (RADAR_MAX_DELTA_NS is 80ms).
    ri = RadarIndexWriter(metadata_dir=meta, flush_interval=1)
    ri.add_radar_frame(frame_id=1, timestamp_ns=1_150_000_000, session_id="s1")
    ri.flush()

    # list_sessions()/session discovery requires a raw/ subfolder to exist,
    # same as every real session DatasetWriter creates.
    (root / "raw").mkdir(parents=True, exist_ok=True)


def test_offline_sync_matches_and_residuals(tmp_path):
    _make_session(tmp_path)
    n = synchronize_session(tmp_path, session_id="s1")
    assert n == 3

    parts = glob.glob(
        str(tmp_path / "s1" / "metadata" / "synchronization" / "**" / "*.parquet"),
        recursive=True)
    assert parts
    df = pd.concat(pd.read_parquet(p) for p in parts).sort_values("camera_frame_id")
    assert len(df) == 3

    row0 = df.iloc[0]
    assert row0["imu_timestamp_ns"] == 1_000_002_000
    assert row0["dt_camera_imu_ns"] == 2000
    assert bool(row0["imu_in_tolerance"])

    # Radar: nearest to 1.0s is 1.15s -> 150ms residual, over the 80ms tolerance.
    assert row0["dt_camera_radar_ns"] == 150_000_000
    assert not bool(row0["radar_in_tolerance"])

    row1 = df.iloc[1]
    # Radar: nearest to 1.1s is 1.15s -> 50ms residual, within tolerance.
    assert row1["dt_camera_radar_ns"] == 50_000_000
    assert bool(row1["radar_in_tolerance"])

    # No lidar/gnss streams were recorded for this session — never crashes,
    # never drops the camera row, just reports "no match".
    assert row0["lidar_timestamp_ns"] == 0
    assert row0["dt_camera_lidar_ns"] == -1
    assert not bool(row0["lidar_in_tolerance"])


def test_offline_sync_defaults_to_most_recently_modified_session(tmp_path):
    _make_session(tmp_path, session_id="s1")
    n = synchronize_session(tmp_path)  # no session_id given
    assert n == 3
    parts = glob.glob(
        str(tmp_path / "s1" / "metadata" / "synchronization" / "**" / "*.parquet"),
        recursive=True)
    assert parts


def test_nearest_match_empty_array_reports_no_match():
    matched, dt = _nearest_match(np.array([], dtype=np.int64), 12345)
    assert matched == 0
    assert dt == -1
