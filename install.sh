#!/usr/bin/env bash
# One-command installer for stereo-dataset.
#
#   ./install.sh
#
# Creates .venv, installs the package and all Python dependencies, and
# prints the available commands. Live capture additionally requires the
# ZED SDK (https://www.stereolabs.com) installed on the system.
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
if ! command -v "$PY" >/dev/null 2>&1; then
    echo "ERROR: $PY not found. Install Python >= 3.10 first." >&2
    exit 1
fi

echo "==> Creating virtualenv"
"$PY" -m venv .venv
. .venv/bin/activate
python -m pip install --upgrade pip

echo "==> Installing stereo-dataset"
pip install .

echo
echo "==> Installed. Commands:"
echo "    .venv/bin/record          Launch the Ground Control Station dashboard"
echo "    .venv/bin/gcs             Same as record"
echo "    .venv/bin/gcs --demo      Dashboard with simulated data (no hardware needed)"
echo "    validate-dataset <dir>   Validate a recorded dataset"
echo "    playback <dir>     Playback a recorded dataset"
echo
if ! python -c "import pyzed" >/dev/null 2>&1; then
    echo "NOTE: pyzed not importable. Install the ZED SDK for live capture;"
    echo "      everything else works without it."
fi
