"""Regression test for the 1970-01-01 filename/partition bug: radar/MTi/GNSS
timestamps are host CLOCK_MONOTONIC, not a UTC epoch. Writing them straight
into datetime.fromtimestamp() (used for filenames and Parquet date
partitions) produced dates near the Unix epoch instead of the real
recording date. The fix requires callers to convert via a captured
mono->wall offset before handing a timestamp to DatasetWriter."""

import glob
import sys
import time
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

from dataset.clock import capture_wall_offset_ns, mono_to_wall_ns
from dataset.writer import DatasetWriter
from radar.frame import RadarFrame, RadarPoint

_CURRENT_YEAR = time.gmtime().tm_year


def test_mono_to_wall_matches_current_date():
    offset = capture_wall_offset_ns()
    mono_now = time.monotonic_ns()
    wall = mono_to_wall_ns(mono_now, offset)
    # Must land within a few seconds of "now" in wall-clock terms, not 1970.
    assert abs(wall - time.time_ns()) < 5_000_000_000


def test_radar_frame_lands_in_current_year_partition_not_1970(tmp_path):
    offset = capture_wall_offset_ns()
    mono_ts = time.monotonic_ns()  # small-ish number, NOT a UTC epoch

    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True,
                           enable_radar=True)
    frame = RadarFrame(
        points=[RadarPoint(x=1.0, y=2.0, z=0.0, velocity_mps=0.5)],
        range_doppler=np.ones((4, 4), dtype=np.uint16),
        range_profile=None,
        timestamp_ns=mono_ts,
        frame_id=1,
        num_objects=1,
        num_range_bins=4,
        num_doppler_bins=4,
    )

    # Calling write_radar_frame WITHOUT a wall_timestamp_ns (the old,
    # buggy call shape) must not silently produce a 1970 date either — but
    # the correct/expected usage always supplies one, which is what we
    # assert here.
    writer.write_radar_frame(frame, wall_timestamp_ns=mono_to_wall_ns(mono_ts, offset))
    writer.close()

    parts = glob.glob(str(tmp_path / "metadata" / "radar" / "**" / "*.parquet"),
                      recursive=True)
    assert parts
    df = pd.concat(pd.read_parquet(p) for p in parts)
    dt = pd.to_datetime(df.iloc[0]["timestamp_ns"], unit="ns", utc=True)
    assert dt.year == _CURRENT_YEAR

    points_files = list((tmp_path / "radar" / "points").glob("*.json"))
    assert points_files
    assert not points_files[0].name.startswith("1970")


def test_write_radar_frame_without_wall_timestamp_falls_back_to_raw(tmp_path):
    """Backward-compat: omitting wall_timestamp_ns must not crash — it
    just falls back to the (possibly 1970-dated) raw value rather than
    raising, since a plausible-but-wrong date beats a dead writer thread."""
    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True,
                           enable_radar=True)
    frame = RadarFrame(
        points=[RadarPoint(x=1.0, y=2.0, z=0.0, velocity_mps=0.5)],
        range_doppler=None, range_profile=None,
        timestamp_ns=1_000_000_000, frame_id=1, num_objects=1,
        num_range_bins=0, num_doppler_bins=0,
    )
    writer.write_radar_frame(frame)  # no wall_timestamp_ns
    writer.close()
    assert writer.write_errors == 0
