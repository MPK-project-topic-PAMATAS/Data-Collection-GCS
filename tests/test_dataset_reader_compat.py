"""dataset/reader.py's free-function API (detect_format, load_v2_frames,
load_v2_mti) must keep working — playback.py and validate_dataset.py import
it directly and previously broke with an ImportError when it was rewritten
to a lidar-only class."""

import sys
import importlib
from types import SimpleNamespace, ModuleType

import h5py
import numpy as np
import pytest

if not hasattr(sys.modules.get("pyzed"), "sl"):
    zed_module = ModuleType("pyzed")
    zed_module.sl = SimpleNamespace(
        RESOLUTION=SimpleNamespace(HD720=object()),
        DEPTH_MODE=SimpleNamespace(NEURAL_LIGHT=object()),
        UNIT=SimpleNamespace(METER=object()),
        COORDINATE_SYSTEM=SimpleNamespace(RIGHT_HANDED_Z_UP=object()),
    )
    sys.modules["pyzed"] = zed_module

from dataset.parquet_index import FrameIndexWriter
from dataset.reader import detect_format, load_v2_frames, load_v2_mti, DatasetReader


def _make_v2_session(root):
    meta = root / "metadata"
    fi = FrameIndexWriter(metadata_dir=meta, flush_interval=1)
    fi.add_frame(
        frame_id=0, timestamp_ns=1_000_000_000, session_id="s1",
        left_path="", right_path="", depth_path="", disparity_path="",
        nearest_mti_ts=0, sync_delta_ns=-1, sensor_hdf5_file="",
        nearest_mti_index=-1,
    )
    fi.flush()

    sensors_dir = root / "sensors"
    sensors_dir.mkdir(parents=True, exist_ok=True)
    with h5py.File(str(sensors_dir / "test_mti.h5"), "w") as f:
        g = f.create_group("mti")
        g.create_dataset("timestamp_ns", data=np.array([1_000_000_500], dtype=np.int64))
        g.create_dataset("acceleration", data=np.array([[0.1, 0.2, 9.8]], dtype=np.float32))
        g.create_dataset("angular_velocity", data=np.array([[0.0, 0.0, 0.01]], dtype=np.float32))


def test_detect_format_v2(tmp_path):
    _make_v2_session(tmp_path)
    assert detect_format(tmp_path) == "v2"


def test_detect_format_unknown(tmp_path):
    assert detect_format(tmp_path) == "unknown"


def test_load_v2_frames_and_mti(tmp_path):
    _make_v2_session(tmp_path)
    frames = load_v2_frames(tmp_path)
    assert len(frames) == 1
    assert frames[0]["frame_id"] == 0

    mti = load_v2_mti(tmp_path)
    assert 1_000_000_500 in mti
    assert mti[1_000_000_500]["az"] == pytest.approx(9.8, abs=1e-5)


def test_dataset_reader_class_wraps_same_logic(tmp_path):
    _make_v2_session(tmp_path)
    reader = DatasetReader(tmp_path)
    assert reader.detect_format() == "v2"
    assert len(reader.frames()) == 1
    assert 1_000_000_500 in reader.mti_samples()


def test_playback_and_validate_dataset_import_cleanly():
    # These previously raised ImportError against a lidar-only reader.py.
    sys.modules.pop("playback", None)
    sys.modules.pop("validate_dataset", None)
    playback = importlib.import_module("playback")
    validate_dataset = importlib.import_module("validate_dataset")
    assert hasattr(playback, "main")
    assert hasattr(validate_dataset, "detect_format")
