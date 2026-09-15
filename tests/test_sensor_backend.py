"""Sensor backend shim must discover the native module and expose its API.

Guards the non-trivial logic in sensor_backend.py: locating the compiled
_sensor_backend .so and re-exporting it under the top-level name.
"""

import importlib
import os
import sys


def test_backend_available_and_exposes_api():
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    try:
        sb = importlib.import_module("sensor_backend")
    finally:
        sys.path.remove(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

    assert sb.available() is True
    # All documented public names must resolve.
    for name in ("AcquisitionConfig", "AcquisitionManager", "SensorDiagnostics",
                 "ZEDConfig", "OusterConfig", "RadarConfig", "MTi8Config",
                 "GNSSConfig", "ZedFrame", "OusterFrame", "RadarFrame",
                 "RadarPoint", "MTiFrame", "GNSSFix"):
        assert hasattr(sb, name), f"missing {name}"

    # Inert construction + lifecycle must not raise.
    manager = sb.AcquisitionManager(sb.AcquisitionConfig())
    assert manager.get_statistics() == {}


def test_shim_requires_build_hint_on_missing_backend():
    """Without the .so, require() raises an actionable ImportError."""
    old_lib = getattr(sys.modules.get("sensor_backend"), "_AVAILABLE", True)
    # Only meaningful when backend is genuinely absent; otherwise skip.
    from sensor_backend import require
    try:
        require()
        assert old_lib  # backend present, nothing to test
    except ImportError as e:
        assert "cmake --build build" in str(e)
