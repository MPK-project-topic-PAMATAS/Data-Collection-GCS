#!/usr/bin/env python3
"""One-command, cross-platform installer for stereo-dataset.

    python install.py

Creates .venv, installs the package and all Python dependencies, and
prints the available commands. Works identically on Linux, macOS, and
Windows — same command, no shell-specific script needed. Live capture
additionally requires the ZED SDK (https://www.stereolabs.com) installed
on the system; pyzed isn't on PyPI so it can't be installed by this
script (see README.md).
"""
from __future__ import annotations

import subprocess
import sys
import venv
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV_DIR = ROOT / ".venv"


def _bin(name: str) -> Path:
    """Path to an executable inside .venv, Windows (Scripts/) vs POSIX (bin/)."""
    if sys.platform == "win32":
        return VENV_DIR / "Scripts" / f"{name}.exe"
    return VENV_DIR / "bin" / name


def main() -> int:
    if sys.version_info < (3, 10):
        print("ERROR: Python >= 3.10 required.", file=sys.stderr)
        return 1

    print("==> Creating virtualenv")
    venv.EnvBuilder(with_pip=True, clear=False).create(VENV_DIR)

    python = _bin("python")
    subprocess.run([str(python), "-m", "pip", "install", "--upgrade", "pip"], check=True)

    print("\n==> Installing stereo-dataset")
    subprocess.run([str(python), "-m", "pip", "install", str(ROOT)], check=True)

    activate = (
        r".venv\Scripts\activate" if sys.platform == "win32" else "source .venv/bin/activate"
    )
    print(f"""
==> Installed. Activate with:
    {activate}

    Commands (after activating, or via .venv/bin|Scripts/<name>):
    record / gcs          Launch the Ground Control Station dashboard
    gcs --demo             Dashboard with simulated data (no hardware needed)
    validate-dataset <dir> Validate a recorded dataset
    playback <dir>         Playback a recorded dataset
""")

    check = subprocess.run(
        [str(python), "-c", "import pyzed"], capture_output=True
    )
    if check.returncode != 0:
        print(
            "NOTE: pyzed not importable. Install the ZED SDK for live capture;\n"
            "      everything else works without it."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
