"""The writer thread must never die from a single bad item, and session
metadata must survive a hard crash between begin_session() and
end_session() — both directly address "the app crashed and I lost data"."""

import sys
import time
from types import SimpleNamespace, ModuleType

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
from dataset.session import SessionWriter
import numpy as np


def _good_frame(frame_id=1):
    return Frame(
        frame_id=frame_id, timestamp_ns=1_000_000_000 + frame_id,
        left=np.zeros((2, 2, 3), dtype=np.uint8),
        right=np.zeros((2, 2, 3), dtype=np.uint8),
        depth=np.zeros((2, 2), dtype=np.float32),
        wall_timestamp_ns=1_700_000_000_000_000_000 + frame_id,
    )


def test_bad_camera_item_does_not_kill_writer_thread(tmp_path):
    writer = DatasetWriter(root=tmp_path, save_depth=True,
                           enable_hdf5=False, enable_parquet=True)

    # A malformed payload: None where a Frame is expected. _write_camera
    # will raise AttributeError trying to read cam.wall_timestamp_ns.
    writer._enqueue("camera", (None, None))

    # The writer thread must survive that and keep processing later items.
    writer.write_camera_frame(_good_frame(2))

    deadline = time.monotonic() + 2.0
    while writer.write_errors < 1 and time.monotonic() < deadline:
        time.sleep(0.02)

    assert writer._thread.is_alive()
    assert writer.write_errors >= 1
    writer.close()

    # The good frame after the bad one was still written.
    assert len(list((tmp_path / "depth").glob("*.npy"))) == 1


def test_session_open_row_persisted_before_stop(tmp_path):
    """If the process crashes between begin_session() and end_session(),
    there must still be a Parquet row proving the session started."""
    meta_dir = tmp_path / "metadata"
    sw = SessionWriter(metadata_dir=meta_dir)
    sw.begin_session(session_id="s1", timestamp_ns=1_700_000_000_000_000_000)

    # Simulate a crash: never call end_session().
    parts = list((meta_dir / "sessions").glob("**/*.parquet"))
    assert parts, "begin_session() must write an OPEN row immediately"

    import pyarrow.parquet as pq
    table = pq.read_table(str(parts[0]))
    row = table.to_pylist()[0]
    assert row["session_id"] == "s1"
    assert row["status"] == "OPEN"
