"""Python shim over the native C++ acquisition backend.

The heavy multithreaded sensor acquisition lives in C++ (``cpp/``), compiled to a
pybind11 module ``_sensor_backend``. This module locates and re-exports it so the
rest of the Python app can ``from sensor_backend import AcquisitionManager``
without caring where the native library lives.

No sensors run just by importing this module. Contructing an
``AcquisitionManager(cfg)`` is also inert until ``start()`` is called.
"""

from __future__ import annotations

import ctypes
import os
import sys

__all__ = [
    "AcquisitionConfig",
    "AcquisitionManager",
    "SensorDiagnostics",
    "ZEDConfig",
    "OusterConfig",
    "RadarConfig",
    "MTi8Config",
    "GNSSConfig",
    "ZedFrame",
    "OusterFrame",
    "RadarFrame",
    "RadarPoint",
    "MTiFrame",
    "GNSSFix",
]

_AVAILABLE = False
_build_hint = (
    "Native backend not built. Build it with:\n"
    "  cd cpp && \\\n"
    "  /home/vgtu/.pyenv/versions/3.10.17/bin/cmake \\\n"
    "    -DPYBIND11_PYTHON_VERSION=3.10 \\\n"
    "    -DCMAKE_PREFIX_PATH=/home/vgtu/.pyenv/versions/3.10.17/lib/python3.10/"
    "site-packages/pybind11/share/cmake/pybind11 \\\n"
    "    -S . -B build && \\\n"
    "  /home/vgtu/.pyenv/versions/3.10.17/bin/cmake --build build -j$(nproc)"
)

_syslibs = ("libsl_zed.so", "libxsensdeviceapi.so", "libxstypes.so")


def _import_native(dir_path: str, name: str):
    """Import ``name`` from ``dir_path`` (and leave it importable later)."""
    sys.path.insert(0, dir_path)
    try:
        return __import__(name)
    finally:
        # Keep it importable for the `from _sensor_backend import ...` below.
        if dir_path not in sys.path:
            sys.path.insert(0, dir_path)


def _find_native_lib() -> str | None:
    """Return path to the compiled _sensor_backend*.so, or None if missing."""
    candidates = [os.path.abspath(os.path.join(os.path.dirname(__file__), "cpp", "build"))]
    env = os.environ.get("SENSOR_BACKEND_DIR")
    if env:
        candidates.insert(0, env)
    for directory in candidates:
        if not os.path.isdir(directory):
            continue
        # Preload SDK runtime libs that live next to the module so its deps resolve.
        for lib in _syslibs:
            p = os.path.join(directory, lib)
            if os.path.exists(p):
                try:
                    ctypes.CDLL(p, mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass
        for name in os.listdir(directory):
            if name.startswith("_sensor_backend") and name.endswith(".so"):
                return os.path.join(directory, name)
    return None


_lib_path = _find_native_lib()
if _lib_path:
    try:
        _native = _import_native(os.path.dirname(_lib_path), "_sensor_backend")
        _AVAILABLE = True
    except ImportError as e:  # pragma: no cover - platform specific
        raise ImportError(
            f"Found native lib {_lib_path} but could not import it: {e}. "
            "It may have been built for a different Python version."
        ) from None
else:
    _native = None


def available() -> bool:
    """True when the native backend is importable."""
    return _AVAILABLE


def require():
    """Raise if the native backend is unavailable; else return the module."""
    if not _AVAILABLE:
        raise ImportError(_build_hint)
    return _native


# Re-export the native public API. Names resolve to the compiled module when
# available; otherwise they are undefined and any use fails with a clear build
# hint from require().
if _AVAILABLE:
    from _sensor_backend import (  # noqa: F401
        AcquisitionConfig,
        AcquisitionManager,
        GNSSConfig,
        GNSSFix,
        MTi8Config,
        MTiFrame,
        OusterConfig,
        OusterFrame,
        RadarConfig,
        RadarFrame,
        RadarPoint,
        SensorDiagnostics,
        ZEDConfig,
        ZedFrame,
    )

    __version__ = "native"
else:  # pragma: no cover - exercised only when backend is not built
    def __getattr__(name):
        raise AttributeError(f"{name!r} requires the native backend. {_build_hint}")

    __version__ = "unavailable"
