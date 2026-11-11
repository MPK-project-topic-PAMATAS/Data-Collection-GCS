"""Estimate per-sensor time offsets from a recorded session.

All timestamps are host-arrival stamps, so each sensor has its own latency.
This tool measures them against the MTi-8 (reference) using physical signals
that every sensor sees:

  * ZED IMU vs MTi-8:  |gyro|  (invariant to the unknown IMU-IMU rotation)
  * Radar vs MTi-8:    Doppler ego-speed vs |MTi velocity| (NED, GNSS-aided)
  * Optional --signal NAME=file.npz: any external (t_s, x) series, e.g. lidar
    angular rate from a lidar-odometry run, compared against |MTi gyro|.

Each pair is estimated over the whole session and again in N windows
(drift check).  A constant offset gives matching windows; a trend is clock
drift; a large spread is jitter that no single constant can remove (then use
hardware sync - see docs/SYNCHRONIZATION.md).

Record a calibration drive with accelerations/decelerations and turns: a
constant-speed straight line makes every signal flat and the offset
unobservable (the tool reports low confidence rather than a number).

Usage:
    python calibration/estimate_time_offsets.py <dataset_root> \
        [--session SID] [--windows 4] [--out offsets.json]
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

import h5py
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fusion.timesync import (  # noqa: E402
    OffsetEstimate, drift_check, estimate_offset, gyro_magnitude,
    radar_ego_speed,
)


def _session_root(root: Path, session: str | None) -> Path:
    if session:
        cand = root / session
        return cand if cand.exists() else root
    return root


def load_h5_group(root: Path, group: str, fields: list[str]) -> dict[str, np.ndarray]:
    """Concatenate ``fields`` of ``/group`` across every sensors/*.h5 file."""
    out: dict[str, list] = {f: [] for f in fields}
    for p in sorted((root / "sensors").glob("*.h5")):
        with h5py.File(str(p), "r") as f:
            if group not in f:
                continue
            g = f[group]
            for k in fields:
                out[k].append(np.asarray(g[k][:]))
    return {k: (np.concatenate(v) if v else np.empty(0)) for k, v in out.items()}


def load_radar_speed(root: Path) -> tuple[np.ndarray, np.ndarray]:
    files = sorted(glob.glob(str(root / "metadata" / "radar" / "**" / "*.parquet"),
                             recursive=True))
    if not files:
        return np.empty(0), np.empty(0)
    df = pd.concat((pd.read_parquet(f) for f in files), ignore_index=True)
    df = df[df["points_path"].astype(bool)].sort_values("timestamp_ns")
    ts, sp = [], []
    for t, rel in zip(df["timestamp_ns"], df["points_path"]):
        try:
            pts = json.loads((root / rel).read_text())["points"]
        except (OSError, ValueError, KeyError):
            continue
        arr = np.array([[p["x"], p["y"], p["z"], p["velocity_mps"]] for p in pts],
                       float)
        v = radar_ego_speed(arr)
        if v is not None:
            ts.append(int(t))
            sp.append(v)
    return np.asarray(ts, np.int64), np.asarray(sp, float)


def _fmt(e: OffsetEstimate) -> str:
    flag = "OK " if e.reliable else "LOW"
    return (f"{e.offset_s * 1e3:+8.2f} ms  corr={e.peak_corr:.2f} "
            f"conf={e.confidence:.2f} [{flag}]")


def analyse_pair(name: str, t_ref, x_ref, t_b, x_b, windows: int,
                 max_lag_s: float, rate_hz: float) -> dict:
    kw = dict(max_lag_s=max_lag_s, rate_hz=rate_hz)
    whole = estimate_offset(t_ref, x_ref, t_b, x_b, **kw)
    wins = drift_check(t_ref, x_ref, t_b, x_b, n_windows=windows, **kw)
    offs = np.array([w.offset_s for w in wins if w.reliable])
    print(f"\n{name}\n  whole session: {_fmt(whole)}")
    for i, w in enumerate(wins):
        print(f"  window {i + 1}/{len(wins)}:   {_fmt(w)}")
    spread = float(offs.max() - offs.min()) if len(offs) > 1 else float("nan")
    if len(offs) > 1:
        print(f"  spread across reliable windows: {spread * 1e3:.2f} ms")
    return {
        "offset_s": whole.offset_s, "reliable": whole.reliable,
        "peak_corr": whole.peak_corr, "confidence": whole.confidence,
        "windows_s": [w.offset_s for w in wins],
        "windows_reliable": [w.reliable for w in wins],
        "spread_s": spread,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path)
    ap.add_argument("--session", default=None)
    ap.add_argument("--windows", type=int, default=4)
    ap.add_argument("--max-lag", type=float, default=0.5, help="seconds")
    ap.add_argument("--rate", type=float, default=100.0, help="resample Hz")
    ap.add_argument("--signal", action="append", default=[],
                    metavar="NAME=FILE.npz",
                    help="external series with arrays t_s and x; compared to "
                         "|MTi gyro|")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)

    root = _session_root(args.root, args.session)
    mti = load_h5_group(root, "mti", ["timestamp_ns", "angular_velocity", "velocity"])
    if mti["timestamp_ns"].size < 100:
        print("error: no MTi data in", root, file=sys.stderr)
        return 2
    t_mti = mti["timestamp_ns"].astype(np.float64) * 1e-9
    gyro_mti = gyro_magnitude(mti["angular_velocity"])
    results: dict[str, dict] = {}

    zed = load_h5_group(root, "zed_imu", ["timestamp_ns", "angular_velocity"])
    if zed["timestamp_ns"].size >= 100:
        t_z = zed["timestamp_ns"].astype(np.float64) * 1e-9
        results["zed_imu_vs_mti"] = analyse_pair(
            "ZED IMU |gyro| vs MTi |gyro|   (positive => ZED stamped late)",
            t_mti, gyro_mti, t_z, gyro_magnitude(zed["angular_velocity"]),
            args.windows, args.max_lag, args.rate)
    else:
        print("\nZED IMU: no data recorded, skipped")

    vel = mti["velocity"]
    if vel.size and np.isfinite(vel).all() and np.abs(vel).sum() > 0:
        t_r, v_r = load_radar_speed(root)
        if t_r.size >= 30:
            results["radar_vs_mti"] = analyse_pair(
                "Radar Doppler ego-speed vs |MTi velocity|  (positive => radar late)",
                t_mti, np.linalg.norm(vel, axis=1),
                t_r.astype(np.float64) * 1e-9, np.abs(v_r),
                args.windows, args.max_lag, args.rate)
        else:
            print("\nRadar: too few frames with usable static points, skipped")
    else:
        print("\nMTi velocity not available (no GNSS-aided fix?); radar skipped")

    for spec in args.signal:
        name, _, fn = spec.partition("=")
        d = np.load(fn)
        results[name + "_vs_mti"] = analyse_pair(
            f"{name} vs MTi |gyro|  (positive => {name} stamped late)",
            t_mti, gyro_mti, np.asarray(d["t_s"], float), np.asarray(d["x"], float),
            args.windows, args.max_lag, args.rate)

    if not results:
        print("nothing to estimate", file=sys.stderr)
        return 2
    print("\nCorrect a sensor by SUBTRACTING its offset from its timestamps.")
    if args.out:
        args.out.write_text(json.dumps(results, indent=2))
        print("wrote", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
