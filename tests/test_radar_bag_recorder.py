import sys
import tempfile
import time
from types import SimpleNamespace

sys.path.insert(0, ".")

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

from dataset.radar_bag_recorder import RadarBagRecorder, RadarBagPoller


class _P:
    def __init__(self, x, y, z, v):
        self.x, self.y, self.z, self.velocity_mps = x, y, z, v


class _F:
    def __init__(self, pts, n):
        self.points = pts
        self.timestamp_ns = 1_000_000_000_000 + n
        self.frame_id = n


class _FakeRadar:
    """Produces 3 unique frames then holds the last (tests poller dedup)."""

    def __init__(self):
        self._n = 0
        self._last = None

    def get_frame(self):
        if self._n < 3:
            self._last = _F([_P(float(self._n), 1, 2, 3.5)], self._n)
            self._n += 1
        return self._last


def _count_msgs(path):
    ts = get_typestore(Stores.ROS1_NOETIC)
    rd = Reader(path)
    rd.open()
    msgs = {}
    for c, t, raw in rd.messages():
        msgs.setdefault(c.topic, []).append(ts.deserialize_ros1(raw, c.msgtype))
    rd.close()
    return msgs


def test_bag_roundtrip_and_poller_dedup():
    path = tempfile.mktemp(suffix=".bag")
    rec = RadarBagRecorder()
    rec.open(path)
    pol = RadarBagPoller(_FakeRadar(), rec, poll_interval_s=0.02)
    pol.start()
    time.sleep(0.3)
    pol.close()

    assert rec.frame_count == 3, "poller must capture each unique frame once"

    msgs = _count_msgs(path)
    assert set(msgs) == {"/radar/points", "/clock"}
    assert len(msgs["/radar/points"]) == 3
    assert len(msgs["/clock"]) == 3

    pc = msgs["/radar/points"][0]
    assert pc.header.frame_id == "radar"
    assert pc.width == 1
    import numpy as np
    pts = np.frombuffer(pc.data, dtype=np.float32).reshape(-1, 4)
    assert pts[0].tolist()[3] == 3.5

    # Wall/epoch timestamp (offset applied) must be a sane recent epoch time.
    assert pc.header.stamp.sec > 1_600_000_000
