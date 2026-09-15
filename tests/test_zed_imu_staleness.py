"""ZED IMU diagnostic staleness detection.

The ZED-2's onboard IMU is read over a separate USB HID interface from the
video feed. Kernel logs showed that interface glitching/re-enumerating on
every camera open (a UVC protocol error immediately followed by a HID
re-enumeration). During that window, ZEDCamera.get_sensors_data() can keep
returning SUCCESS with a cached, non-advancing sample — so the dashboard
silently displayed a frozen "diagnostic" reading with no indication
anything was wrong (reported: "zed imu got stuck mid point"). This is
purely cosmetic — MTi (navigation) and recording are unaffected, and raw
ZED IMU is captured inside the SVO2 regardless — but a frozen number
presented as live is misleading. Fixed by tracking the ZED SDK's own
per-sample IMU timestamp and flagging staleness after 1s without a change.
"""

import sys
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

from dashboard.acquisition import AcquisitionThread


def _make_thread():
    return AcquisitionThread.__new__(AcquisitionThread)


def test_missing_zed_imu_is_stale():
    t = _make_thread()
    t._last_zed_imu_ts = 0
    t._last_zed_imu_change_time = 0.0

    accel, gyro, quat, stale = t._read_zed_imu(None)
    assert stale is True
    assert accel == [0.0, 0.0, 0.0]
    assert quat == [0.0, 0.0, 0.0, 1.0]


def test_advancing_timestamp_is_not_stale():
    t = _make_thread()
    t._last_zed_imu_ts = 0
    t._last_zed_imu_change_time = 0.0  # far in the past

    zi = {
        "linear_acceleration": [0.1, 0.2, 9.8],
        "angular_velocity": [0.0, 0.0, 0.01],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "timestamp": 123456789,
    }
    accel, gyro, quat, stale = t._read_zed_imu(zi)
    assert stale is False, "a fresh, never-before-seen timestamp must reset staleness"
    assert accel == [0.1, 0.2, 9.8]
    assert t._last_zed_imu_ts == 123456789


def test_repeated_identical_timestamp_becomes_stale():
    """Simulates the actual bug: get_sensors_data() keeps succeeding but
    the SDK returns the exact same cached sample every call."""
    t = _make_thread()
    t._last_zed_imu_ts = 0
    t._last_zed_imu_change_time = 0.0  # already 1s+ in the past at test time

    zi = {
        "linear_acceleration": [0.1, 0.2, 9.8],
        "angular_velocity": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "timestamp": 999,  # same value every call
    }
    # First call: timestamp is "new" relative to the initial 0 -> not stale yet.
    _, _, _, stale1 = t._read_zed_imu(zi)
    assert stale1 is False

    # Force the clock backward artificially by resetting the change-time to
    # simulate real time passing without a new sample ever arriving.
    t._last_zed_imu_change_time -= 2.0
    _, _, _, stale2 = t._read_zed_imu(zi)
    assert stale2 is True, "an unchanging timestamp for >1s must be flagged stale"
