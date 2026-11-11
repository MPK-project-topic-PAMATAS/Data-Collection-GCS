import json
import sys
from pathlib import Path

import h5py
import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
import estimate_time_offsets as eto  # noqa: E402


def _walk(t, seed):
    rng = np.random.default_rng(seed)
    k = np.cumsum(rng.normal(size=300))
    return np.interp(t, np.linspace(t.min(), t.max(), 300), k)


def _make_session(root: Path, zed_late=0.031, radar_late=0.08):
    (root / "sensors").mkdir(parents=True)
    t = np.arange(0, 90, 0.005)                      # MTi 200 Hz
    w = _walk(t, 1)
    gyro = np.c_[w * 0.3, w * 0.5, w * 0.1].astype(np.float32)
    speed = 8 + 2 * np.sin(t / 7) + 0.5 * _walk(t, 2) / 10
    vel = np.c_[speed, 0 * speed, 0 * speed].astype(np.float32)
    t0 = 10_000_000_000
    with h5py.File(root / "sensors" / "a.h5", "w") as f:
        g = f.create_group("mti")
        g["timestamp_ns"] = (t * 1e9).astype(np.int64) + t0
        g["angular_velocity"] = gyro
        g["velocity"] = vel
        tz = np.arange(0, 90, 1 / 400.0)             # ZED IMU 400 Hz, different rot.
        wz = np.interp(tz - zed_late, t, w)
        z = f.create_group("zed_imu")
        z["timestamp_ns"] = (tz * 1e9).astype(np.int64) + t0
        z["angular_velocity"] = np.c_[0.5 * wz, 0.1 * wz, 0.3 * wz].astype(np.float32)
    # radar 10 Hz: points on a static scene seen at ego speed
    (root / "radar" / "points").mkdir(parents=True)
    (root / "metadata" / "radar").mkdir(parents=True)
    rt = np.arange(1, 89, 0.1)
    rng = np.random.default_rng(3)
    rows = []
    for i, tt in enumerate(rt):
        v = float(np.interp(tt - radar_late, t, speed))
        ang = rng.uniform(-1, 1, 30)
        r = rng.uniform(5, 40, 30)
        pts = [{"x": float(a), "y": float(b), "z": 0.0,
                "velocity_mps": float(-v * b / c)}
               for a, b, c in zip(r * np.sin(ang), r * np.cos(ang), r)]
        rel = f"radar/points/{i}.json"
        (root / rel).write_text(json.dumps({"points": pts}))
        rows.append({"timestamp_ns": int(tt * 1e9) + t0, "points_path": rel})
    pd.DataFrame(rows).to_parquet(root / "metadata" / "radar" / "p.parquet")


def test_cli_recovers_injected_latencies(tmp_path, capsys):
    _make_session(tmp_path, zed_late=0.031, radar_late=0.08)
    out = tmp_path / "o.json"
    assert eto.main([str(tmp_path), "--out", str(out), "--windows", "3"]) == 0
    res = json.loads(out.read_text())
    assert res["zed_imu_vs_mti"]["offset_s"] == pytest.approx(0.031, abs=0.003)
    assert res["zed_imu_vs_mti"]["reliable"]
    assert res["radar_vs_mti"]["offset_s"] == pytest.approx(0.08, abs=0.01)
    assert res["radar_vs_mti"]["spread_s"] < 0.02       # constant offset -> no drift


def test_cli_no_mti_fails(tmp_path):
    (tmp_path / "sensors").mkdir()
    assert eto.main([str(tmp_path)]) == 2
