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

from dataset.config_snapshot import build_config_snapshot


def test_config_snapshot_has_required_sections_and_no_secrets():
    snapshot = build_config_snapshot(
        camera_calib={"width": 1280, "height": 720, "fps": 30},
        mti=SimpleNamespace(running=True, firmware="1.0"),
        radar=SimpleNamespace(),
        lidar=None,
        gnss=SimpleNamespace(),
        session_id="20260101_000000",
    )

    for key in ("session_id", "software_version", "git_commit", "host_clock",
                "camera", "mti", "radar", "lidar", "gnss", "synchronization"):
        assert key in snapshot

    assert snapshot["host_clock"]["ptp_implemented"] is False
    assert snapshot["host_clock"]["gps_pps_implemented"] is False

    assert snapshot["camera"]["effective"]["width"] == 1280

    # No NTRIP username/password anywhere in the snapshot.
    assert "password" not in snapshot["gnss"]["ntrip"]
    assert "username" not in snapshot["gnss"]["ntrip"]
    for caster in snapshot["gnss"]["ntrip"].get("casters", []):
        assert "username" not in caster
        assert "password" not in caster
