"""Prove recording is native-rate and independent per sensor:

  - MTi/radar/GNSS structured pollers persist every native sample even when
    zero camera frames are ever written.
  - A camera frame with no matching IMU/radar/lidar/gnss still produces a
    metadata/frames row (recording is never gated on synchronization).
"""

import sys
import time
import glob
from types import SimpleNamespace, ModuleType

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

from dataset.writer import DatasetWriter
from dataset.frame import Frame
from dataset.mti_structured_recorder import MTiStructuredPoller
from dataset.radar_structured_recorder import RadarStructuredPoller
from dataset.gnss_structured_recorder import GNSSStructuredPoller
from mti8.frame import MTiFrame
from radar.frame import RadarFrame, RadarPoint


class _FakeSource:
    """Minimal get_new_since()-based fake, matching MTiReceiver/RadarReceiver."""

    def __init__(self, items):
        self._items = items

    def get_new_since(self, after_ts_ns):
        return [i for i in self._items if i.timestamp_ns > after_ts_ns]


class _FakeGNSS:
    def __init__(self, fixes):
        self._fixes = fixes

    def get_new_since(self, after_ts_ns):
        return [f for f in self._fixes if f["timestamp_ns"] > after_ts_ns]


def _wait_for(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_mti_native_rate_recording_needs_no_camera_frame(tmp_path):
    # Timestamps must be genuinely "in the future" relative to
    # time.monotonic_ns() at poller.start() — the poller now (correctly)
    # skips anything buffered/latest at start time, so it doesn't replay a
    # pre-recording backlog (see mti_structured_recorder.py's start()).
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=True, enable_parquet=False)
    source = _FakeSource([])
    poller = MTiStructuredPoller(source, writer, poll_interval_s=0.01)
    poller.start()  # captures the start-time cursor before any frames exist
    # Generous spacing (50ms) so appending the frame list after start()
    # (unavoidable — the poller must exist first) can never be mistaken for
    # pre-recording backlog by the fix that skips anything at/before the
    # start-time cursor.
    base = time.monotonic_ns()
    source._items = [
        MTiFrame(timestamp_ns=base + i * 50_000_000, packet_counter=i,
                acceleration=(0.0, 0.0, 9.81), gyroscope=(0.0, 0.0, 0.0))
        for i in range(1, 11)
    ]
    try:
        _wait_for(lambda: writer.mti_samples_written >= 10)
    finally:
        poller.stop()
    writer.close()

    # Zero camera frames were ever written, yet every native MTi packet
    # reached HDF5 — removing the camera does not remove an IMU sample.
    assert writer.mti_samples_written == 10


def test_radar_native_rate_recording_needs_no_camera_frame(tmp_path):
    base = time.monotonic_ns()
    frames = [
        RadarFrame(
            points=[RadarPoint(x=1.0, y=2.0, z=0.0, velocity_mps=0.5)],
            range_doppler=np.ones((4, 4), dtype=np.uint16),
            range_profile=None,
            timestamp_ns=base + i * 150_000_000,
            frame_id=i,
            num_objects=1,
            num_range_bins=4,
            num_doppler_bins=4,
        )
        for i in range(1, 6)
    ]
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True,
                           enable_radar=True)
    poller = RadarStructuredPoller(_FakeSource(frames), writer, poll_interval_s=0.01)
    poller.start()
    try:
        _wait_for(lambda: len(list((tmp_path / "radar" / "points").glob("*.json"))) >= 5)
    finally:
        poller.stop()
    writer.close()

    # No camera frame was written; radar was recorded at its own rate anyway.
    assert len(list((tmp_path / "radar" / "points").glob("*.json"))) == 5
    radar_parts = glob.glob(str(tmp_path / "metadata" / "radar" / "**" / "*.parquet"),
                            recursive=True)
    assert radar_parts
    df = pd.concat(pd.read_parquet(p) for p in radar_parts)
    assert len(df) == 5


def test_gnss_native_rate_recording_needs_no_camera_frame(tmp_path):
    base = time.monotonic_ns()
    fixes = [
        {"timestamp_ns": base + i * 200_000_000, "lat": 1.0, "lon": 2.0, "alt": 3.0,
         "fix": "3D Fix", "fix_quality": 1, "satellites": 8, "hdop": 1.0,
         "speed": 0.0, "course": 0.0, "rtk_fixed": False, "utc_time": "",
         "gps_week": 0, "gps_tow": 0.0}
        for i in range(1, 4)
    ]
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True)
    poller = GNSSStructuredPoller(_FakeGNSS(fixes), writer, poll_interval_s=0.01)
    poller.start()
    try:
        def _has_rows():
            parts = glob.glob(str(tmp_path / "metadata" / "gnss" / "**" / "*.parquet"),
                              recursive=True)
            return bool(parts)
        _wait_for(_has_rows)
    finally:
        poller.stop()
    writer.close()

    gnss_parts = glob.glob(str(tmp_path / "metadata" / "gnss" / "**" / "*.parquet"),
                           recursive=True)
    assert gnss_parts
    df = pd.concat(pd.read_parquet(p) for p in gnss_parts)
    assert len(df) == 3


def test_mti_poller_skips_pre_recording_backlog(tmp_path):
    """Regression for a real bug found by an external dataset audit:
    MTiReceiver runs continuously from app startup, so by the time REC is
    pressed its ring buffer can already hold up to buffer_size samples from
    before recording began. The poller's cursor used to start at 0, so the
    first get_new_since(0) call replayed that whole backlog into the
    session's HDF5 file (observed: ~128 rows timestamped before session
    start). Only samples arriving after poller.start() must be captured.
    """
    now = time.monotonic_ns()
    accel = (0.0, 0.0, 9.81)
    gyro = (0.0, 0.0, 0.0)
    old_frames = [MTiFrame(timestamp_ns=now - (10 - i) * 1_000_000, packet_counter=i,
                          acceleration=accel, gyroscope=gyro)
                  for i in range(10)]  # all strictly in the past
    source = _FakeSource(list(old_frames))

    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=True, enable_parquet=False)
    poller = MTiStructuredPoller(source, writer, poll_interval_s=0.01)
    poller.start()
    time.sleep(0.1)  # let a few polls happen against the stale backlog

    # Now append genuinely new samples and confirm only those are captured.
    new_base = time.monotonic_ns()
    new_frames = [MTiFrame(timestamp_ns=new_base + i * 1_000_000, packet_counter=100 + i,
                          acceleration=accel, gyroscope=gyro)
                  for i in range(1, 4)]
    source._items = old_frames + new_frames

    try:
        _wait_for(lambda: writer.mti_samples_written >= 3)
    finally:
        poller.stop()
    writer.close()

    assert writer.mti_samples_written == 3, (
        "pre-recording backlog leaked into the session — "
        f"got {writer.mti_samples_written}, expected exactly the 3 new samples"
    )


def test_non_inertial_mti_packet_not_written_as_fake_zero_sample(tmp_path):
    """Regression for a real bug found by an external dataset audit: some
    Xsens output configurations emit GNSS-PVT-only packets with no
    acceleration/gyroscope fields. mti_frame_to_sample() falls back to
    [0, 0, 0] for an absent field, so these were being written to HDF5 as
    indistinguishable-from-real physically-zero IMU samples. They must be
    skipped from the structured HDF5 stream entirely (raw Xbus still has
    them) rather than fabricated as zero readings.
    """
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=True, enable_parquet=False)
    gnss_only = MTiFrame(timestamp_ns=1_000_000, packet_counter=1,
                         acceleration=None, gyroscope=None,
                         latitude=54.68, longitude=25.27)
    inertial = MTiFrame(timestamp_ns=2_000_000, packet_counter=2,
                        acceleration=(0.0, 0.0, 9.81), gyroscope=(0.0, 0.0, 0.0))

    writer.write_mti_sample(gnss_only, host_rx_timestamp_ns=1_700_000_000_000_000_000)
    writer.write_mti_sample(inertial, host_rx_timestamp_ns=1_700_000_000_100_000_000)
    writer.close()

    assert writer.mti_samples_written == 1, (
        "the GNSS-only (non-inertial) packet must not produce an HDF5 row"
    )


def test_camera_frame_recorded_without_any_sensor_match(tmp_path):
    """Recording a camera frame must not require a synchronized match."""
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True)
    frame = Frame(
        frame_id=1, timestamp_ns=1_000_000_000,
        left=np.zeros((4, 4, 3), dtype=np.uint8),
        right=np.zeros((4, 4, 3), dtype=np.uint8),
        depth=np.zeros((4, 4), dtype=np.float32),
    )
    writer.write_camera_frame(frame, synced_preview=None)
    writer.close()

    parts = glob.glob(str(tmp_path / "metadata" / "frames" / "**" / "*.parquet"),
                      recursive=True)
    assert parts
    df = pd.concat(pd.read_parquet(p) for p in parts)
    assert len(df) == 1
    assert df.iloc[0]["frame_id"] == 1
    assert df.iloc[0]["sync_delta_ns"] == -1
