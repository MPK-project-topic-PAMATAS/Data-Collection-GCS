import sys
from types import SimpleNamespace
from types import ModuleType
import time

# The system pytest interpreter does not include the ZED SDK.  Radar receiver
# tests need only its configuration constants, so provide the tiny compatible
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

from radar.receiver import RadarReceiver


def test_streaming_requires_open_port_and_recent_complete_frame():
    receiver = RadarReceiver()
    receiver._data_reader = SimpleNamespace(
        _serial=SimpleNamespace(is_open=True)
    )

    receiver._last_frame_ns = time.monotonic_ns()
    assert receiver.connected
    assert receiver.streaming

    receiver._last_frame_ns = 0
    assert not receiver.streaming


def test_streaming_is_false_for_closed_data_port():
    receiver = RadarReceiver()
    receiver._data_reader = SimpleNamespace(
        _serial=SimpleNamespace(is_open=False)
    )
    receiver._last_frame_ns = time.monotonic_ns()

    assert not receiver.connected
    assert not receiver.streaming
