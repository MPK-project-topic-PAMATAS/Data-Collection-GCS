"""SVO compression mode must be validated against what the installed SDK
actually exposes — fail loudly on an unknown mode, never silently ignore it
or fall back to an unconfigured default."""

import sys
from types import SimpleNamespace, ModuleType

import pytest

# camera/zed_camera.py needs a fuller pyzed.sl surface than other tests'
# minimal stub (RESOLUTION/DEPTH_MODE/UNIT/COORDINATE_SYSTEM only), so build
# the full stub unconditionally — it must win regardless of import order,
# since sys.modules["pyzed"] may already exist from an earlier test file.
_svo_modes = SimpleNamespace(
    H264=object(), H264_LOSSLESS=object(),
    H265=object(), H265_LOSSLESS=object(),
    LOSSLESS=object(),
)


class _FakeCamera:
    def __init__(self):
        self.last_recording_params = None

    def enable_recording(self, rec_params):
        self.last_recording_params = rec_params
        return _zed_sl.ERROR_CODE.SUCCESS

    def is_opened(self):
        return True


class _FakeRecordingParameters:
    def __init__(self):
        self.video_filename = ""
        self.compression_mode = None


_zed_sl = SimpleNamespace(
    RESOLUTION=SimpleNamespace(HD720=object()),
    DEPTH_MODE=SimpleNamespace(NEURAL_LIGHT=object()),
    UNIT=SimpleNamespace(METER=object()),
    COORDINATE_SYSTEM=SimpleNamespace(RIGHT_HANDED_Z_UP=object()),
    SVO_COMPRESSION_MODE=_svo_modes,
    ERROR_CODE=SimpleNamespace(SUCCESS=object()),
    RecordingParameters=_FakeRecordingParameters,
    Camera=_FakeCamera,
    InitParameters=lambda: SimpleNamespace(),
    RuntimeParameters=lambda: SimpleNamespace(),
    Mat=lambda: SimpleNamespace(),
    SensorsData=lambda: SimpleNamespace(),
)

zed_module = ModuleType("pyzed")
zed_module.sl = _zed_sl
sys.modules["pyzed"] = zed_module

import camera.zed_camera as zed_camera_module  # noqa: E402
from camera.zed_camera import ZEDCamera, _SVO_COMPRESSION_MODES  # noqa: E402


def _bare_camera():
    cam = ZEDCamera()
    cam.calib = {}
    return cam


def test_valid_compression_mode_resolves_to_sdk_member(monkeypatch, tmp_path):
    monkeypatch.setattr(zed_camera_module, "SVO_COMPRESSION_MODE", "H265_LOSSLESS")
    cam = _bare_camera()
    cam.start_recording(str(tmp_path / "out.svo2"))
    assert cam.camera.last_recording_params.compression_mode is _svo_modes.H265_LOSSLESS
    assert cam.calib["svo_compression_mode"] == "H265_LOSSLESS"


def test_unknown_compression_mode_raises_value_error(monkeypatch, tmp_path):
    monkeypatch.setattr(zed_camera_module, "SVO_COMPRESSION_MODE", "NOT_A_REAL_MODE")
    cam = _bare_camera()
    with pytest.raises(ValueError, match="NOT_A_REAL_MODE"):
        cam.start_recording(str(tmp_path / "out.svo2"))
    # Must fail before ever touching the SDK's enable_recording.
    assert cam.camera.last_recording_params is None


def test_compression_mode_table_matches_installed_sdk():
    assert set(_SVO_COMPRESSION_MODES) == {
        "H264", "H264_LOSSLESS", "H265", "H265_LOSSLESS", "LOSSLESS",
    }
