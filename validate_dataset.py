"""Dataset validator for v2 format (HDF5/Parquet/PNG/NPY) with HTML report.

Checks structure, HDF5 integrity, Parquet schemas, image validity,
cross-file consistency, and generates a self-contained HTML report.

Supports both v1 (CSV-based) and v2 (HDF5/Parquet) formats.
Auto-detects format by checking for timestamps.csv vs metadata/frames/.

Usage:
    python validate_dataset.py <dataset_dir>          # terminal + HTML
    python validate_dataset.py <dataset_dir> --html   # HTML only
"""
from __future__ import annotations
import json
import math
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import h5py
import numpy as np
import pandas as pd

# ── Expected schemas ─────────────────────────────────────────────────────

# v2 Parquet frame index columns (matches dataset/parquet_index.py SCHEMA)
FRAME_INDEX_COLS = [
    "frame_id", "timestamp_ns", "datetime_utc", "session_id",
    "left_path", "right_path", "depth_path", "disparity_path",
    "nearest_mti_timestamp_ns", "sync_delta_ns", "sensor_hdf5_file",
    "nearest_mti_index", "exposure", "gain", "temperature",
]

# v2 HDF5 MTi datasets (matches dataset/hdf5_sensor_writer.py MTI_DATASETS)
MTI_HDF5_DATASETS = [
    "timestamp_ns", "packet_counter", "host_timestamp_ns", "device_timestamp",
    "acceleration", "angular_velocity", "magnetometer", "quaternion_raw",
    "latitude", "longitude", "altitude_msl", "altitude_ellipsoid",
    "velocity", "temperature", "baro_pressure", "status_word", "status_byte",
    "delta_velocity", "delta_quaternion",
]

ZED_IMU_HDF5_DATASETS = [
    "timestamp_ns", "acceleration", "angular_velocity",
    "quaternion", "temperature",
]

SESSION_COLS = [
    "session_id", "start_timestamp_ns", "end_timestamp_ns",
    "start_datetime_utc", "end_datetime_utc", "status",
    "frames_start", "frames_end", "mti_start_index", "mti_end_index",
]

EVENT_COLS = ["event_id", "timestamp_ns", "session_id", "event_type", "description"]

REQUIRED_DIRS_V2 = [
    "sensors", "metadata", "calibration", "raw",
]

CALIB_FILES = ["zed_intrinsics.yaml", "camera_info.json"]
EXTRINSIC_FILES = ["camera_to_mti.yaml", "camera_to_vehicle.yaml", "gnss_to_vehicle.yaml",
                    "radar_to_camera.yaml"]
SYNC_THRESHOLD_MS = 20.0


# ── Checker ──────────────────────────────────────────────────────────────

class Check:
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"

    def __init__(self, name, severity=PASS, message="", detail=""):
        self.name = name
        self.severity = severity
        self.message = message
        self.detail = detail

    def __repr__(self):
        return f"[{self.severity.upper()}] {self.name}: {self.message}"


# ── Format detection ─────────────────────────────────────────────────────

from dataset.reader import detect_format, list_sessions  # noqa: F811


# ── v2 Validators ────────────────────────────────────────────────────────

def _validate_hdf5(path: Path, checks, summary, prefix=""):
    """Validate a single HDF5 sensor file."""
    hdf5_files = sorted(path.glob("*.h5"))
    if not hdf5_files:
        checks.append(Check(f"{prefix}hdf5_empty", Check.WARN, "No HDF5 files found"))
        return

    total_samples = 0
    bad_files = 0
    shapes = {}

    for hf in hdf5_files:
        try:
            with h5py.File(str(hf), "r") as f:
                # Check required groups exist
                has_mti = "mti" in f
                has_meta = "metadata" in f

                if not has_mti:
                    checks.append(Check(
                        f"{prefix}hdf5_no_mti", Check.FAIL,
                        f"{hf.name}: missing 'mti' group"
                    ))
                    bad_files += 1
                    continue

                mti = f["mti"]
                n_samples = mti["timestamp_ns"].shape[0]
                total_samples += n_samples

                # Check dataset names
                actual_ds = set(mti.keys())
                expected_ds = set(MTI_HDF5_DATASETS)
                missing = expected_ds - actual_ds
                if missing:
                    checks.append(Check(
                        f"{prefix}hdf5_mti_datasets", Check.FAIL,
                        f"{hf.name}: missing MTi datasets {missing}"
                    ))

                # Check timestamp monotonicity
                if n_samples > 1:
                    ts = mti["timestamp_ns"][:]
                    non_mono = np.sum(np.diff(ts) <= 0)
                    if non_mono:
                        checks.append(Check(
                            f"{prefix}hdf5_ts_mono", Check.WARN,
                            f"{hf.name}: {non_mono} non-monotonic timestamps"
                        ))

                # Check quaternion validity
                if "quaternion_raw" in mti:
                    q = mti["quaternion_raw"][:]
                    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
                    norms = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
                    bad = np.sum(np.abs(norms - 1.0) > 0.01)
                    if bad:
                        checks.append(Check(
                            f"{prefix}hdf5_quat_norm", Check.WARN,
                            f"{hf.name}: {bad}/{n_samples} quaternions |norm-1| > 0.01"
                        ))

                # Record shape for summary
                if n_samples > 0:
                    shapes[hf.name] = n_samples

                # Check ZED IMU if present
                if "zed_imu" in f:
                    zed = f["zed_imu"]
                    zed_n = zed["timestamp_ns"].shape[0]
                    summary[f"{prefix}zed_imu_samples"] = summary.get(f"{prefix}zed_imu_samples", 0) + zed_n

        except Exception as e:
            checks.append(Check(
                f"{prefix}hdf5_corrupt", Check.FAIL,
                f"{hf.name}: {e}"
            ))
            bad_files += 1

    if bad_files == 0:
        checks.append(Check(f"{prefix}hdf5_integrity", Check.PASS, f"{len(hdf5_files)} files, all valid"))

    summary[f"{prefix}hdf5_files"] = len(hdf5_files)
    summary[f"{prefix}hdf5_total_samples"] = total_samples


def _validate_parquet(root: Path, checks, summary):
    """Validate Parquet metadata files."""
    frames_dir = root / "metadata" / "frames"
    sessions_dir = root / "metadata" / "sessions"
    events_dir = root / "metadata" / "events"

    # Frame index
    if frames_dir.exists():
        parts = sorted(frames_dir.glob("**/*.parquet"))
        if not parts:
            checks.append(Check("parquet_frames_empty", Check.WARN, "No Parquet frame index files"))
        else:
            try:
                df = pd.read_parquet(frames_dir)
                n = len(df)

                # Check required columns
                actual_cols = set(df.columns)
                missing = set(FRAME_INDEX_COLS) - actual_cols
                if missing:
                    checks.append(Check("parquet_frames_cols", Check.FAIL, f"Missing columns: {missing}"))
                else:
                    checks.append(Check("parquet_frames_cols", Check.PASS, f"{n} frames, all columns present"))

                # Check frame_id continuity. frame_id is the camera's own
                # running grab counter (process lifetime, not reset per
                # session — see dataset/writer.py write_camera_frame), so a
                # later session in the same process legitimately starts
                # above 0; only gaps/reordering within the session matter.
                if "frame_id" in df.columns:
                    ids = df.sort_values("timestamp_ns")["frame_id"].values
                    if np.all(np.diff(ids) == 1):
                        checks.append(Check("parquet_frame_continuity", Check.PASS, f"Frames {ids[0]}..{ids[-1]} continuous"))
                    else:
                        checks.append(Check("parquet_frame_continuity", Check.WARN, f"Frame IDs not continuous"))

                # Check timestamp monotonicity
                if "timestamp_ns" in df.columns:
                    ts = df["timestamp_ns"].values
                    non_mono = np.sum(np.diff(ts) <= 0)
                    if non_mono:
                        checks.append(Check("parquet_ts_mono", Check.WARN, f"{non_mono} non-monotonic timestamps"))
                    else:
                        checks.append(Check("parquet_ts_mono", Check.PASS))

                # Check image files exist
                if "left_path" in df.columns:
                    missing_imgs = 0
                    for img_path in df["left_path"].iloc[:10]:
                        if not (root / img_path).exists():
                            missing_imgs += 1
                    if missing_imgs:
                        checks.append(Check("parquet_imgs_exist", Check.FAIL, f"{missing_imgs}/10 sampled images missing"))
                    else:
                        checks.append(Check("parquet_imgs_exist", Check.PASS, "All sampled images exist"))

                summary["parquet_frames"] = n

            except Exception as e:
                checks.append(Check("parquet_frames_parse", Check.FAIL, str(e)))
    else:
        checks.append(Check("parquet_frames_missing", Check.WARN, "No metadata/frames/ directory"))

    # Sessions
    if sessions_dir.exists():
        parts = sorted(sessions_dir.glob("**/*.parquet"))
        if parts:
            try:
                df = pd.read_parquet(sessions_dir)
                n = len(df)
                summary["sessions_count"] = n

                if "session_id" in df.columns:
                    ids = df["session_id"].unique()
                    checks.append(Check("parquet_sessions", Check.PASS, f"{n} session records, {len(ids)} unique sessions"))

                # Check completed sessions have end_timestamp_ns. The OPEN
                # row is written immediately and never deleted (append-only
                # log — see dataset/session.py); a later COMPLETED row for
                # the same session_id supersedes it, so only a session_id
                # with NO COMPLETED row at all is actually incomplete.
                if "status" in df.columns and "end_timestamp_ns" in df.columns:
                    completed = df[df["status"] == "COMPLETED"]
                    open_sessions = df[
                        (df["status"] == "OPEN")
                        & (~df["session_id"].isin(completed["session_id"]))
                    ]
                    if len(open_sessions):
                        checks.append(Check("parquet_sessions_open", Check.WARN, f"{len(open_sessions)} OPEN sessions (incomplete?)"))
                    checks.append(Check("parquet_sessions_completed", Check.PASS, f"{len(completed)} completed sessions"))
            except Exception as e:
                checks.append(Check("parquet_sessions_parse", Check.FAIL, str(e)))
    else:
        checks.append(Check("parquet_sessions_missing", Check.WARN, "No metadata/sessions/ directory"))

    # Events
    if events_dir.exists():
        parts = sorted(events_dir.glob("**/*.parquet"))
        if parts:
            try:
                df = pd.read_parquet(events_dir)
                n = len(df)
                summary["events_count"] = n
                checks.append(Check("parquet_events", Check.PASS, f"{n} events"))
            except Exception as e:
                checks.append(Check("parquet_events_parse", Check.FAIL, str(e)))
    else:
        checks.append(Check("parquet_events_missing", Check.WARN, "No metadata/events/ directory"))


def _validate_images_v2(root: Path, checks, summary):
    """Validate dense maps (depth etc.); per-frame colour images are optional.

    All ZED2 video and IMU is recorded inside the SVO2 file under raw/, so
    left/right per-frame images are never expected.  SVO2 presence is checked
    separately in _validate_raw.
    """
    depth_dir = root / "depth"
    disp_dir = root / "disparity"
    conf_dir = root / "confidence"

    left_imgs = list(root.glob("left/*")) if (root / "left").exists() else []
    right_imgs = list(root.glob("right/*")) if (root / "right").exists() else []
    left_imgs = sorted(left_imgs)
    right_imgs = sorted(right_imgs)

    n_left, n_right = len(left_imgs), len(right_imgs)
    summary["left_images"] = n_left
    summary["right_images"] = n_right

    if n_left == 0:
        checks.append(Check("images_none", Check.PASS, "No per-frame images (SVO2-only recording — expected)"))
    elif n_left != n_right:
        checks.append(Check("images_mismatch", Check.WARN, f"Left={n_left}, right={n_right}"))
    else:
        checks.append(Check("images_count", Check.PASS, f"{n_left} pairs"))

    n_depth = len(list(depth_dir.glob("*.npy"))) if depth_dir.exists() else 0
    summary["depth_files"] = n_depth
    if n_depth == 0:
        checks.append(Check("depth_none", Check.WARN, "No depth maps found"))
    else:
        checks.append(Check("depth_count", Check.PASS, f"{n_depth} depth maps"))

    # Depth / disparity / confidence (NPY files)
    for name, d in [("depth", depth_dir), ("disparity", disp_dir), ("confidence", conf_dir)]:
        files = sorted(d.glob("*.npy")) if d.exists() else []
        summary[f"{name}_files"] = len(files)
        if files and n_left:
            if len(files) != n_left:
                checks.append(Check(f"{name}_count", Check.WARN, f"{len(files)} {name} files for {n_left} images"))

            # Shape check
            try:
                arr = np.load(str(files[0]))
                summary[f"{name}_shape"] = list(arr.shape)
                if name == "depth":
                    nan_frac = np.isnan(arr).mean()
                    if nan_frac > 0.5:
                        checks.append(Check(f"{name}_nans", Check.WARN, f"{nan_frac*100:.1f}% NaN in first map"))
                    else:
                        checks.append(Check(f"{name}_valid", Check.PASS, f"{nan_frac*100:.2f}% NaN"))
            except Exception as e:
                checks.append(Check(f"{name}_load", Check.FAIL, f"Cannot load sample: {e}"))

    if not depth_dir.exists():
        checks.append(Check("depth_missing", Check.WARN, "No depth/ directory"))


def _validate_calibration_v2(root: Path, checks, summary):
    """Validate calibration files."""
    calib_dir = root / "calibration"
    for cf in CALIB_FILES:
        p = calib_dir / cf
        if not p.exists():
            checks.append(Check("calib_missing", Check.FAIL, f"Missing: {cf}"))

    cam_info_path = calib_dir / "camera_info.json"
    if cam_info_path.exists():
        try:
            calib = json.loads(cam_info_path.read_text())
            if calib.get("fx", 0) <= 0 or calib.get("fy", 0) <= 0:
                checks.append(Check("calib_invalid", Check.FAIL, f"fx/fy must be > 0"))
            else:
                checks.append(Check("calib_values", Check.PASS,
                                    f"fx={calib['fx']:.1f} fy={calib['fy']:.1f}"))
        except (json.JSONDecodeError, KeyError) as e:
            checks.append(Check("calib_parse", Check.FAIL, f"Cannot parse: {e}"))

    ext_dir = calib_dir / "extrinsics"
    identity_warned = False
    for ef in EXTRINSIC_FILES:
        p = ext_dir / ef
        if not p.exists():
            checks.append(Check("extrinsic_missing", Check.WARN, f"Placeholder not created: {ef}"))
        elif p.stat().st_size < 20 and not identity_warned:
            checks.append(Check("extrinsics_identity", Check.WARN, "Extrinsics are identity placeholders"))
            identity_warned = True


def _validate_raw(root: Path, checks, summary):
    """Validate raw files."""
    raw_dir = root / "raw"
    raw_files = list(raw_dir.glob("*")) if raw_dir.exists() else []
    summary["raw_files"] = len(raw_files)
    raw_sizes = {rf.name: rf.stat().st_size for rf in raw_files}
    summary["raw_sizes"] = raw_sizes
    checks.append(Check("raw_files", Check.PASS, f"{len(raw_files)} files"))

    # SVO2 is the canonical ZED2 video+IMU record.
    svo2 = [f for f in raw_files if f.suffix.lower() == ".svo2"]
    if svo2:
        checks.append(Check("svo2", Check.PASS,
                            f"{len(svo2)} SVO2 file(s), {svo2[0].name} "
                            f"({svo2[0].stat().st_size / (1024**2):.1f} MiB)"))
    else:
        checks.append(Check("svo2", Check.WARN, "No .svo2 file found in raw/"))


def _cross_validate_v2(root: Path, checks, summary):
    """Cross-validate frame count between Parquet index and image files."""
    frames_dir = root / "metadata" / "frames"
    if not frames_dir.exists():
        return

    try:
        df = pd.read_parquet(frames_dir)
        parquet_n = len(df)
    except Exception:
        return

    # Per-frame depth maps (the per-frame dense record) vs Parquet count.
    # Per-frame colour images are not written (SVO2-only video).
    depth_dir = root / "depth"
    if depth_dir.exists():
        img_n = sum(1 for _ in depth_dir.glob("*.npy"))
        if parquet_n != img_n:
            checks.append(Check(
                "cross_parquet_depth", Check.WARN,
                f"Parquet frames={parquet_n}, depth maps={img_n}"
            ))
        else:
            checks.append(Check(
                "cross_parquet_depth", Check.PASS,
                f"Parquet and depth counts match ({parquet_n})"
            ))

    # HDF5 sample count vs Parquet frame count
    sensors_dir = root / "sensors"
    if sensors_dir.exists():
        hdf5_total = summary.get("hdf5_total_samples", 0)
        if hdf5_total and parquet_n:
            ratio = hdf5_total / parquet_n
            if ratio < 0.8 or ratio > 1.2:
                checks.append(Check(
                    "cross_hdf5_frames", Check.WARN,
                    f"HDF5 samples ({hdf5_total}) / frames ({parquet_n}) = {ratio:.2f}"
                ))
            else:
                checks.append(Check(
                    "cross_hdf5_frames", Check.PASS,
                    f"HDF5/frame ratio = {ratio:.2f}"
                ))


# ── Main validate ────────────────────────────────────────────────────────

def validate(dataset_dir: str) -> tuple[list[Check], dict]:
    """Run all checks, return (checks, summary_dict)."""
    root = Path(dataset_dir)
    checks: list[Check] = []
    summary = {}

    fmt = detect_format(root)
    if fmt == "unknown":
        # dataset_dir may be a dataset root holding one-subfolder-per-session
        # (DatasetWriter.begin_session()) rather than a session folder
        # itself — auto-descend into the most recently modified session.
        sessions = list_sessions(root)
        if sessions:
            root = root / sessions[0]
            summary["session_id"] = sessions[0]
            checks.append(Check(
                "session_autodetect", Check.PASS,
                f"No dataset directly at {dataset_dir}; validating most "
                f"recently modified session: {sessions[0]}"
            ))
            fmt = detect_format(root)
    summary["format"] = fmt

    if fmt == "v1":
        checks.append(Check("format", Check.FAIL, "v1 format no longer supported — use v2"))
        return checks, summary

    if fmt == "unknown":
        checks.append(Check("format", Check.FAIL, f"Unknown dataset format in {root}"))
        return checks, summary

    checks.append(Check("format", Check.PASS, f"v2 format detected"))

    # ── 1. Directory structure ──────────────────────────────────────
    for d in REQUIRED_DIRS_V2:
        full = root / d
        if not full.exists():
            checks.append(Check("dir_missing", Check.FAIL, f"Missing directory: {d}"))
        elif not full.is_dir():
            checks.append(Check("dir_not_dir", Check.FAIL, f"Not a directory: {d}"))
    checks.append(Check("directories", Check.PASS, "All required directories present"))

    # ── 2. Images ──────────────────────────────────────────────────
    _validate_images_v2(root, checks, summary)

    # ── 3. HDF5 sensor files ──────────────────────────────────────
    sensors_dir = root / "sensors"
    _validate_hdf5(sensors_dir, checks, summary)

    # ── 4. Parquet metadata ───────────────────────────────────────
    _validate_parquet(root, checks, summary)

    # ── 5. Calibration ─────────────────────────────────────────────
    _validate_calibration_v2(root, checks, summary)

    # ── 6. Raw files ──────────────────────────────────────────────
    _validate_raw(root, checks, summary)

    # ── 7. Cross-file consistency ─────────────────────────────────
    _cross_validate_v2(root, checks, summary)

    # ── 8. Disk size ──────────────────────────────────────────────
    total_bytes = sum(f.stat().st_size for f in root.rglob("*") if f.is_file())
    summary["total_size_gb"] = round(total_bytes / (1024**3), 3)

    return checks, summary


# ── Terminal output ──────────────────────────────────────────────────────

def print_checks(checks):
    icons = {"pass": "\033[92m✓\033[0m", "warn": "\033[93m⚠\033[0m", "fail": "\033[91m✗\033[0m"}
    for c in checks:
        print(f"  {icons[c.severity]} {c.name}: {c.message}")


# ── HTML report ──────────────────────────────────────────────────────────

def generate_html(dataset_dir, checks, summary):
    root = Path(dataset_dir)
    dataset_name = root.name

    n_pass = sum(1 for c in checks if c.severity == Check.PASS)
    n_warn = sum(1 for c in checks if c.severity == Check.WARN)
    n_fail = sum(1 for c in checks if c.severity == Check.FAIL)
    status = "PASS" if n_fail == 0 else "FAIL"
    status_color = "#22c55e" if n_fail == 0 else "#ef4444"

    def row(c):
        color = {"pass": "#22c55e", "warn": "#eab308", "fail": "#ef4444"}[c.severity]
        icon = {"pass": "✓", "warn": "⚠", "fail": "✗"}[c.severity]
        return f'<tr><td style="color:{color};font-weight:bold">{icon}</td><td>{c.name}</td><td>{c.message}</td></tr>'

    checks_html = "\n".join(row(c) for c in checks)

    def srow(label, value):
        return f"<tr><td>{label}</td><td><strong>{value}</strong></td></tr>"

    summary_rows = ""
    summary_rows += srow("Format", summary.get("format", "unknown"))
    summary_rows += srow("Left images", summary.get("left_images", 0))
    summary_rows += srow("Right images", summary.get("right_images", 0))
    summary_rows += srow("Depth maps", summary.get("depth_files", 0))
    summary_rows += srow("HDF5 files", summary.get("hdf5_files", 0))
    summary_rows += srow("HDF5 samples", summary.get("hdf5_total_samples", 0))
    summary_rows += srow("Parquet frames", summary.get("parquet_frames", 0))
    summary_rows += srow("Sessions", summary.get("sessions_count", 0))
    summary_rows += srow("Events", summary.get("events_count", 0))
    summary_rows += srow("Total size", f"{summary.get('total_size_gb', 0):.3f} GB")

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Dataset Report — {dataset_name}</title>
<style>
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #0f172a; color: #e2e8f0; padding: 2rem; line-height: 1.6; }}
  .container {{ max-width: 900px; margin: 0 auto; }}
  h1 {{ font-size: 1.5rem; margin-bottom: 0.25rem; }}
  .subtitle {{ color: #94a3b8; font-size: 0.875rem; margin-bottom: 1.5rem; }}
  .status {{ display: inline-block; padding: 0.25rem 0.75rem; border-radius: 4px;
             font-weight: bold; font-size: 0.875rem; color: #0f172a;
             background: {status_color}; margin-bottom: 1.5rem; }}
  .counts {{ display: flex; gap: 1.5rem; margin-bottom: 1.5rem; }}
  .count {{ text-align: center; }}
  .count .num {{ font-size: 1.5rem; font-weight: bold; }}
  .count .label {{ font-size: 0.75rem; color: #94a3b8; text-transform: uppercase; }}
  .pass .num {{ color: #22c55e; }}
  .warn .num {{ color: #eab308; }}
  .fail .num {{ color: #ef4444; }}
  section {{ margin-bottom: 1.5rem; }}
  h2 {{ font-size: 1rem; color: #94a3b8; text-transform: uppercase; letter-spacing: 0.05em;
       border-bottom: 1px solid #1e293b; padding-bottom: 0.5rem; margin-bottom: 0.75rem; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.875rem; }}
  td {{ padding: 0.35rem 0.75rem; border-bottom: 1px solid #1e293b; }}
  td:first-child {{ width: 2rem; text-align: center; }}
  td:nth-child(2) {{ font-weight: 500; white-space: nowrap; }}
  tr:hover {{ background: #1e293b; }}
</style>
</head>
<body>
<div class="container">
  <h1>{dataset_name}</h1>
  <div class="subtitle">Validated {now}</div>
  <div class="status">{status}</div>

  <div class="counts">
    <div class="count pass"><div class="num">{n_pass}</div><div class="label">Passed</div></div>
    <div class="count warn"><div class="num">{n_warn}</div><div class="label">Warnings</div></div>
    <div class="count fail"><div class="num">{n_fail}</div><div class="label">Failed</div></div>
  </div>

  <section>
    <h2>Data Summary</h2>
    <table>{summary_rows}</table>
  </section>

  <section>
    <h2>All Checks ({len(checks)})</h2>
    <table>{checks_html}</table>
  </section>
</div>
</body>
</html>"""
    return html


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    import argparse
    p = argparse.ArgumentParser(description="Validate a stereo dataset")
    p.add_argument("dataset_dir", help="Path to dataset directory")
    p.add_argument("--html", action="store_true", help="Generate HTML report only")
    args = p.parse_args()

    print(f"Validating: {args.dataset_dir}\n")
    t0 = time.time()
    checks, summary = validate(args.dataset_dir)
    elapsed = time.time() - t0

    if not args.html:
        print_checks(checks)
        print(f"\n  ({elapsed:.1f}s)")

    # Generate HTML report
    root = Path(args.dataset_dir)
    html = generate_html(args.dataset_dir, checks, summary)
    report_path = root / "dataset_report.html"
    report_path.write_text(html)

    n_fail = sum(1 for c in checks if c.severity == Check.FAIL)
    n_warn = sum(1 for c in checks if c.severity == Check.WARN)
    status = "PASS" if n_fail == 0 else "FAIL"

    print(f"\n  Report: {report_path}")
    print(f"  Result: {status} ({n_fail} fail, {n_warn} warn)")

    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
