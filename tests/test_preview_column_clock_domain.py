"""Regression for a real bug found by an external dataset audit: the frame
index's nearest_mti_timestamp_ns preview column was the live Synchronizer's
raw host-monotonic IMU timestamp, stored next to this row's own
timestamp_ns which is wall-clock (UTC epoch). The two looked related but
weren't on the same clock — "subtracting the two stored timestamps does not
produce the reported delta." nearest_mti_timestamp_ns must be converted to
the same wall-clock basis as timestamp_ns, using the camera frame's own
simultaneous mono/wall pair, so the two columns are internally consistent.
"""

import sys
import glob
from types import SimpleNamespace, ModuleType

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

import numpy as np

from dataset.writer import DatasetWriter
from dataset.frame import Frame


def test_nearest_mti_timestamp_is_on_the_same_clock_as_timestamp_ns(tmp_path):
    # A camera frame with a known mono<->wall offset.
    cam_mono_ns = 5_000_000_000          # host monotonic at grab()
    cam_wall_ns = 1_700_000_000_000_000_000  # UTC epoch at grab()
    frame = Frame(
        frame_id=1, timestamp_ns=cam_mono_ns,
        left=np.zeros((2, 2, 3), dtype=np.uint8),
        right=np.zeros((2, 2, 3), dtype=np.uint8),
        depth=np.zeros((2, 2), dtype=np.float32),
        wall_timestamp_ns=cam_wall_ns,
    )

    # A live-preview match: IMU sample 3ms (in monotonic terms) after the
    # camera frame, on the SAME (monotonic) clock as cam_mono_ns.
    imu_mono_ns = cam_mono_ns + 3_000_000
    synced_preview = SimpleNamespace(
        imu=object(),  # only needs to be truthy
        imu_timestamp_ns=imu_mono_ns,
        delta_ns=3_000_000,
    )

    writer = DatasetWriter(root=tmp_path, save_depth=False,
                           enable_hdf5=False, enable_parquet=True)
    writer.write_camera_frame(frame, synced_preview=synced_preview)
    writer.close()

    parts = glob.glob(str(tmp_path / "metadata" / "frames" / "**" / "*.parquet"),
                      recursive=True)
    df = pd.concat(pd.read_parquet(p) for p in parts)
    row = df.iloc[0]

    assert row["timestamp_ns"] == cam_wall_ns
    # The stored nearest_mti_timestamp_ns must be wall-clock too, and its
    # difference from this row's own timestamp_ns must equal sync_delta_ns
    # — that consistency is exactly what the audit found broken.
    assert row["sync_delta_ns"] == 3_000_000
    assert row["nearest_mti_timestamp_ns"] - row["timestamp_ns"] == 3_000_000
