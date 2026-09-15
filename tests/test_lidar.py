import sys
from types import SimpleNamespace, ModuleType
from pathlib import Path

import numpy as np

# The system pytest interpreter does not include the ZED SDK.  Writer tests
# need only its configuration constants, so provide the tiny compatible
# surface before config.settings is imported.
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
from dataset.reader import iter_lidar_frames, DatasetReader
from fusion.synchronizer import Synchronizer
from lidar.receiver import LidarReceiver


def _make_camera_frame():
    return Frame(
        frame_id=1,
        timestamp_ns=1_700_000_000_000_000_000,
        left=np.zeros((20, 20, 3), dtype=np.uint8),
        right=np.zeros((20, 20, 3), dtype=np.uint8),
        depth=np.zeros((20, 20), dtype=np.float32),
        wall_timestamp_ns=1_700_000_000_123_456_789,
    )


def test_camera_frame_written_without_lidar_and_images_skipped(tmp_path):
    """DatasetWriter has no lidar write path at all — lidar recording is a
    single pcap + single config JSON, owned entirely by lidar/receiver.py's
    own ingest thread, independent of the camera and of DatasetWriter."""
    writer = DatasetWriter(
        root=tmp_path, save_depth=True, enable_hdf5=False, enable_parquet=False
    )
    writer.write_camera_frame(_make_camera_frame())
    writer.close()

    # DatasetWriter has no lidar write path any more — nothing to assert
    # under tmp_path/lidar; that directory is only ever populated by
    # LidarReceiver itself during a live recording.
    assert not (tmp_path / "lidar").exists() or not list(
        (tmp_path / "lidar").glob("*.json")
    )

    # All ZED2 video/IMU lives in SVO2; no per-frame images are written.
    assert not list((tmp_path / "left").glob("*"))
    assert not list((tmp_path / "right").glob("*"))
    # Depth map is still written regardless of the svo2-only switch.
    assert len(list((tmp_path / "depth").glob("*.npy"))) == 1


def test_latest_lidar_is_available_when_camera_imu_sync_fails():
    """A live dashboard cloud must not depend on the recording sync gate."""
    live = SimpleNamespace(
        timestamp_ns=500_000_000,
        frame_id=7,
        points=np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        points_full=np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        reflectivity=None,
        num_points=1,
        complete=True,
    )

    class NoImu:
        def get_nearest(self, _timestamp_ns):
            return None

    class Lidar:
        def get_frame(self):
            return live

        def get_nearest(self, _timestamp_ns):
            return live

    sync = Synchronizer(NoImu(), lidar_receiver=Lidar())
    camera = SimpleNamespace(timestamp_ns=500_000_000)

    assert sync.sync(camera) is None
    assert sync.latest_lidar_frame() is live


def test_lidar_receiver_has_no_per_frame_json_writer():
    """The per-frame JSON writer must not exist at all — a pcap-only
    recorder can't accidentally regain it via a leftover method."""
    assert not hasattr(LidarReceiver, "_write_json_frame")
    assert not hasattr(LidarReceiver, "json_count")


def test_start_pcap_writes_exactly_one_config_json(tmp_path, monkeypatch):
    """start_pcap() must write exactly one companion JSON, and ingesting
    many frames afterward must not create any additional per-frame files."""
    receiver = LidarReceiver.__new__(LidarReceiver)
    receiver._info = SimpleNamespace(to_json_string=lambda: '{"prod_line": "OS-0-64"}')
    receiver._pcap_handle = None
    receiver._pcap_path = None
    receiver._config_json_path = None
    receiver._pcap_packets = 0

    import ouster.sdk._bindings.pcap as _pcap
    monkeypatch.setattr(_pcap, "record_initialize", lambda *a, **k: object())

    pcap_path = str(tmp_path / "lidar.pcap")
    receiver.start_pcap(pcap_path)

    assert receiver.pcap_path == pcap_path
    assert receiver.config_json_path == str(tmp_path / "lidar_0.json")
    assert (tmp_path / "lidar_0.json").exists()

    # No per-frame JSON directory or files exist, or are ever created by
    # ingesting frames — _write_pcap_frame is the only per-frame hook left.
    monkeypatch.setattr(receiver, "_write_pcap_frame", lambda lf: None)
    receiver._xyz_lut = None  # unused: fake frames report has_field()=False
    receiver._buffer = __import__("collections").deque(maxlen=10)
    receiver._latest = None
    receiver._lock = __import__("threading").Lock()
    receiver._frame_count = 0
    receiver._last_frame_ns = 0
    receiver._start_time = 0
    receiver._rate = 0.0
    receiver._diagnosed = True  # skip the startup diagnostic print
    receiver._prev_frame_id = None
    receiver._dropped_frames = 0

    class _FakeLidarFrame:
        def __init__(self, frame_id):
            self.frame_id = frame_id
        def has_field(self, _f):
            return False
        def field(self, _f):
            return None
        def complete(self):
            return True

    for i in range(5):
        receiver._ingest_frame(_FakeLidarFrame(i))

    json_files = sorted(set(tmp_path.rglob("*.json")))
    assert json_files == [tmp_path / "lidar_0.json"]


def test_iter_lidar_frames_requires_pcap_and_config(tmp_path):
    import pytest as _pytest

    with _pytest.raises(FileNotFoundError):
        list(iter_lidar_frames(tmp_path / "missing.pcap"))

    pcap = tmp_path / "lidar.pcap"
    pcap.write_bytes(b"")
    with _pytest.raises(FileNotFoundError, match="config JSON"):
        list(iter_lidar_frames(pcap))


def test_dataset_reader_reports_zero_lidar_frames_without_pcap(tmp_path):
    reader = DatasetReader(tmp_path)
    assert reader.num_lidar_frames == 0
    assert reader.lidar_frames() == []
    assert reader.get_lidar_frame(0) is None
