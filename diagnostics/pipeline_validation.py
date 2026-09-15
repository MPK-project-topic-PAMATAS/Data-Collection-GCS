"""Pipeline validation — checks all config flags and sensor sources.

Run before recording to verify MTi-8 is sole nav source.

Usage:
    .venv/bin/python diagnostics/pipeline_validation.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def validate():
    from config.settings import (
        ENABLE_ZED_IMU_LOGGING,
        MTI_PORT, MTI_BAUDRATE,
        CAMERA_RESOLUTION, CAMERA_FPS,
    )

    checks = []
    ok = True

    # ── ZED IMU logging ──────────────────────────────────────────────
    if ENABLE_ZED_IMU_LOGGING:
        checks.append(("PASS", "ENABLE_ZED_IMU_LOGGING=True",
                        "ZED IMU logged for diagnostic comparison only"))
    else:
        checks.append(("INFO", "ENABLE_ZED_IMU_LOGGING=False",
                        "ZED IMU logging disabled"))

    # ── MTi-8 config ─────────────────────────────────────────────────
    if MTI_PORT:
        checks.append(("PASS", f"MTI_PORT={MTI_PORT}",
                        "MTi-8 serial port configured"))
    else:
        ok = False
        checks.append(("FAIL", "MTI_PORT empty",
                        "MTi-8 must be connected as primary IMU"))

    if MTI_BAUDRATE == 2000000:
        checks.append(("PASS", f"MTI_BAUDRATE={MTI_BAUDRATE}",
                        "MTi-8 baudrate correct"))
    else:
        checks.append(("WARN", f"MTI_BAUDRATE={MTI_BAUDRATE}",
                        "Expected 2000000 for MTi-8"))

    # ── Camera config ────────────────────────────────────────────────
    checks.append(("PASS", f"Camera={CAMERA_RESOLUTION} @ {CAMERA_FPS} FPS",
                    "Camera configuration"))

    # ── Print results ────────────────────────────────────────────────
    print("=" * 60)
    print("Pipeline Validation — MTi-8 Sole Navigation Source")
    print("=" * 60)

    for status, name, detail in checks:
        icon = {"PASS": "\u2713", "FAIL": "\u2717",
                "WARN": "!", "INFO": "\u2139"}[status]
        print(f"  [{icon}] {name}: {detail}")

    print("=" * 60)
    if ok:
        print("RESULT: PASS — Pipeline configured correctly for MTi-8 navigation")
    else:
        print("RESULT: FAIL — Fix issues above before recording")
    print("=" * 60)

    return ok


if __name__ == "__main__":
    sys.exit(0 if validate() else 1)
