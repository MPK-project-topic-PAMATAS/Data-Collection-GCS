"""Estimate host clock drift/jitter against GPS time from a recorded session.

There is no hardware PPS/PTP anywhere in this pipeline (see
docs/SYNCHRONIZATION.md) — every sensor timestamp is a host-clock arrival stamp,
and dataset/clock.py assumes the host monotonic->wall offset is constant for
the life of a recording. This script checks that assumption using the one
absolute time reference already recorded alongside the data: each GNSS fix's
NMEA UTC time, persisted as gps_week/gps_tow in metadata/gnss (see
gnss/receiver.py's _utc_to_gps_week_tow and dataset/parquet_index.py's
GNSS_SCHEMA).

For every valid fix it pairs (host wall-clock arrival time, GPS time) and
fits a line through them (same idea as INSLIB's inslib_clock_error.py, which
fits a sensor board's free-running counter against GPS time pulses — here
the "counter" is the host's own wall clock). The fitted slope reports the
host clock's drift against GPS time in ppm; the fit residual reports how
much jitter is in GNSS-to-host timestamping (NMEA output-rate quantization,
serial/USB latency, NTP steps) — i.e. how much to trust GNSS timestamps
relative to the rest of the session.

This is a read-only diagnostic. It does not touch recorded data.

Usage:
    python diagnose_clock_sync.py <dataset_root> [--session SESSION_ID]
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from dataset.reader import list_sessions

_GPS_WEEK_SECS = 7 * 24 * 3600


def _read_partitioned_parquet(dirpath: Path) -> pd.DataFrame:
    files = sorted(glob.glob(str(dirpath / "**" / "*.parquet"), recursive=True))
    if not files:
        return pd.DataFrame()
    return pd.concat((pd.read_parquet(f) for f in files), ignore_index=True)


def _fit_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float, np.ndarray]:
    """Least-squares y = slope * x + intercept, fit about the means for
    numerical stability (x/y are large ns-scale integers). Returns
    (slope, intercept_about_means, residuals)."""
    x_mean = x.mean()
    y_mean = y.mean()
    xc = x - x_mean
    yc = y - y_mean
    denom = np.dot(xc, xc)
    if denom == 0:
        return 1.0, 0.0, np.zeros_like(y)
    slope = np.dot(xc, yc) / denom
    residuals = yc - slope * xc
    return slope, y_mean - slope * x_mean, residuals


def clock_drift_report(gnss_df: pd.DataFrame) -> dict | None:
    """Fit host-wall-clock vs GPS time from a session's GNSS fixes.

    Returns None if there are fewer than 2 fixes with a valid GPS time
    (gps_week > 0, i.e. RMC has produced at least one valid fix).
    """
    valid = gnss_df[(gnss_df["gps_week"] > 0) & (gnss_df["fix_quality"] > 0)]
    valid = valid.sort_values("timestamp_ns")
    if len(valid) < 2:
        return None

    host_ns = valid["timestamp_ns"].to_numpy(dtype=np.float64)
    gps_ns = (
        valid["gps_week"].to_numpy(dtype=np.float64) * _GPS_WEEK_SECS
        + valid["gps_tow"].to_numpy(dtype=np.float64)
    ) * 1e9

    slope, _intercept, residuals_ns = _fit_line(host_ns, gps_ns)

    ppm = (slope - 1.0) * 1e6
    rms_ms = float(np.sqrt(np.mean(residuals_ns ** 2))) / 1e6
    max_abs_ms = float(np.max(np.abs(residuals_ns))) / 1e6
    span_s = float(host_ns.max() - host_ns.min()) / 1e9

    return {
        "n_fixes": int(len(valid)),
        "span_s": span_s,
        "drift_ppm": float(ppm),
        "residual_rms_ms": rms_ms,
        "residual_max_ms": max_abs_ms,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", help="Dataset root directory")
    parser.add_argument("--session", default=None, help="Session ID (default: most recent)")
    args = parser.parse_args()

    dataset_root = Path(args.dataset_root)
    session_id = args.session
    if session_id is None:
        sessions = list_sessions(dataset_root)
        if not sessions:
            print(f"No sessions found under {dataset_root}", file=sys.stderr)
            return 1
        session_id = sessions[0]

    gnss_dir = dataset_root / session_id / "metadata" / "gnss"
    gnss_df = _read_partitioned_parquet(gnss_dir)
    if gnss_df.empty:
        print(f"No GNSS data recorded for session {session_id!r}", file=sys.stderr)
        return 1

    report = clock_drift_report(gnss_df)
    if report is None:
        print(
            f"Fewer than 2 fixes with a valid GPS time in session "
            f"{session_id!r} (no RMC fix / all No-Fix) — cannot estimate drift.",
            file=sys.stderr,
        )
        return 1

    print(f"Session:              {session_id}")
    print(f"GNSS fixes used:       {report['n_fixes']}")
    print(f"Span:                  {report['span_s']:.1f} s")
    print(f"Host clock drift:      {report['drift_ppm']:+.2f} ppm vs GPS time")
    print(f"Residual RMS:          {report['residual_rms_ms']:.2f} ms")
    print(f"Residual max:          {report['residual_max_ms']:.2f} ms")

    if report["residual_rms_ms"] > 50.0:
        print(
            "\nWARNING: residual RMS > 50 ms — GNSS timestamps are noisier "
            "than typical NMEA/USB jitter; treat wall-clock alignment for "
            "this session with caution."
        )
    if abs(report["drift_ppm"]) > 200.0:
        print(
            "\nWARNING: drift > 200 ppm — check for an NTP step, GPS week "
            "rollover, or a bad RMC parse during this session."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
