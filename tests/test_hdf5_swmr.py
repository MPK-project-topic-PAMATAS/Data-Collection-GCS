"""Regression for a real bug found by an external dataset audit: a session
marked COMPLETED still had an MTi HDF5 file that failed to open read-only
("file is already open for write") and failed SWMR read too — meaning a
recorded, finished session's structured IMU data was unreadable by any
tool as long as the application process stayed alive (HDF5SensorWriter
lives for the whole app lifetime, not just one session).

Fix: enable HDF5 SWMR (Single-Writer/Multiple-Reader) mode once every
dataset in the file has been created. This test opens a file for reading
with swmr=True WHILE the writer still holds it open for writing — exactly
the scenario the audit's tooling needed and failed at.
"""

import time
from pathlib import Path

import h5py

from dataset.hdf5_sensor_writer import HDF5SensorWriter, mti_frame_to_sample
from mti8.frame import MTiFrame


def test_hdf5_file_is_readable_via_swmr_while_still_open_for_write(tmp_path):
    writer = HDF5SensorWriter(sensors_dir=tmp_path, enable_zed_imu=False)
    writer.start()

    frame = MTiFrame(timestamp_ns=1_700_000_000_000_000_000, packet_counter=1,
                     acceleration=(0.0, 0.0, 9.81), gyroscope=(0.0, 0.0, 0.0))
    sample = mti_frame_to_sample(frame, host_timestamp_ns=frame.timestamp_ns)
    sample.timestamp_ns = frame.timestamp_ns
    writer.put_mti_sample(sample)

    deadline = time.monotonic() + 2.0
    while writer.current_filename is None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert writer.current_filename is not None, "writer never opened a file"

    path = tmp_path / writer.current_filename
    writer.flush()

    try:
        # The direct, environment-independent check that the fix actually
        # took effect: HDF5 file locking behavior for a same-process
        # second open varies by filesystem/platform (this local test
        # environment did not reproduce the audit's exact "already open
        # for write" error even without the fix), so the property that
        # must hold everywhere is that SWMR writing was actually enabled.
        assert writer._current_file.swmr_mode is True

        # And, where the platform's HDF5 build supports it, a concurrent
        # SWMR read must not raise — this is what an external reader/audit
        # tool needs to do while a recording session is still active.
        with h5py.File(str(path), "r", swmr=True) as f:
            assert "mti" in f
            assert f["mti"]["timestamp_ns"].shape[0] >= 1
    finally:
        writer.stop()
