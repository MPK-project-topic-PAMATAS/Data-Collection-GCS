"""Drop/gap detection must be counted, never silently absorbed into a
clean-looking dataset. MTi packet-counter gap tracking is already covered
by tests/test_mti8_validator.py; this covers the new radar/lidar
frame-sequence gap counters."""

import sys
import threading
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

from radar.receiver import RadarReceiver
from lidar.receiver import LidarReceiver


def test_radar_frame_gap_counted():
    r = RadarReceiver()
    assert r.dropped_frames == 0
    for frame_number in (1, 2, 3, 7, 8):  # gap of 3 between 3 and 7
        r._track_frame_gap(frame_number)
    assert r.dropped_frames == 3


def test_radar_frame_gap_ignores_first_sample():
    r = RadarReceiver()
    r._track_frame_gap(42)  # first sample ever — no prior reference
    assert r.dropped_frames == 0


def test_lidar_frame_gap_counted():
    l = LidarReceiver.__new__(LidarReceiver)
    l._lock = threading.Lock()
    l._prev_frame_id = None
    l._dropped_frames = 0
    for frame_id in (0, 1, 2, 5):  # gap of 2 between 2 and 5
        l._track_frame_gap(frame_id)
    assert l.dropped_frames == 2
