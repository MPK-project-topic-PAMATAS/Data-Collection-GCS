"""Regression test: if the camera never opened (or drops), AcquisitionThread
must not keep calling camera.grab() in a near-zero-delay loop. Doing so
floods the ZED SDK with "sl::Camera::Open has not been called" — thousands
of lines per second — which is what a user reported as "the system broke
completely." grab() must not be called at all while the camera reports
not-opened; the loop should back off instead."""

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

from dashboard.acquisition import AcquisitionThread, SharedFrame


class _NeverOpenCamera:
    def __init__(self):
        self.grab_calls = 0
        self.is_opened_calls = 0

    def is_opened(self):
        self.is_opened_calls += 1
        return False

    def grab(self):
        self.grab_calls += 1
        return None


def test_grab_not_called_while_camera_not_opened():
    camera = _NeverOpenCamera()
    shared = SharedFrame()
    thread = AcquisitionThread(camera, synchronizer=object(), writer=object(),
                               shared=shared)
    thread.start()
    try:
        time.sleep(0.3)
    finally:
        thread.stop()
        thread.join(timeout=2.0)

    assert camera.grab_calls == 0, (
        "grab() was called on an unopened camera — this is exactly what "
        "flooded the ZED SDK with 'Camera::Open has not been called' errors"
    )
    # It must still be polling is_opened() so it notices a reconnect.
    assert camera.is_opened_calls >= 1
