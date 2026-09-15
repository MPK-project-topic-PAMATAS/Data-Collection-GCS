#!/usr/bin/env python3
"""Phase 1+2: Diagnose ZED disparity acquisition and validate depth relationship.

Opens camera, grabs 30 frames, prints full disparity statistics,
validates d = f*B/Z relationship, saves results.
"""
import sys
import time
import numpy as np

sys.path.insert(0, ".")

from pyzed import sl
from camera.zed_camera import ZEDCamera


def main():
    zed = ZEDCamera()
    try:
        zed.open()
    except RuntimeError as e:
        print(f"FATAL: {e}")
        return 1

    info = zed.camera.get_camera_information()
    left_cam = info.camera_configuration.calibration_parameters.left_cam
    fx = left_cam.fx
    baseline = info.camera_configuration.calibration_parameters.get_camera_baseline()

    print(f"Camera: {info.camera_model}  S/N {info.serial_number}")
    print(f"fx={fx:.4f}  baseline={baseline:.6f} m")
    print(f"Resolution: {info.camera_configuration.resolution.width}x{info.camera_configuration.resolution.height}")
    print()

    depth_mat = sl.Mat()
    disp_mat = sl.Mat()

    N_FRAMES = 30
    all_disp = []
    all_depth = []

    print(f"Grabbing {N_FRAMES} frames...")
    for i in range(N_FRAMES):
        if zed.camera.grab(zed.runtime) != sl.ERROR_CODE.SUCCESS:
            print(f"  Frame {i}: grab failed")
            continue

        zed.camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
        zed.camera.retrieve_measure(disp_mat, sl.MEASURE.DISPARITY)

        d = depth_mat.get_data().copy()
        disp = disp_mat.get_data().copy()

        all_disp.append(disp)
        all_depth.append(d)

        if i < 3:
            _print_frame_stats(i, d, disp)

    zed.camera.close()

    if not all_disp:
        print("No frames grabbed!")
        return 1

    # ── Aggregate disparity stats ──────────────────────────────
    disp_stack = np.concatenate([d.ravel() for d in all_disp])
    depth_stack = np.concatenate([d.ravel() for d in all_depth])

    print("\n" + "=" * 60)
    print("AGGREGATE DISPARITY STATISTICS (all frames)")
    print("=" * 60)
    _print_full_stats("disparity", disp_stack)

    print("\n" + "=" * 60)
    print("AGGREGATE DEPTH STATISTICS (all frames)")
    print("=" * 60)
    _print_full_stats("depth", depth_stack)

    # ── Phase 2: Mathematical validation ───────────────────────
    print("\n" + "=" * 60)
    print("DEPTH ↔ DISPARITY VALIDATION")
    print(f"d = f*B/Z  (f={fx:.4f}, B={baseline:.6f})")
    print("=" * 60)

    fxB = fx * baseline
    print(f"  f*B = {fxB:.6f}")

    # Collect paired (depth, disparity) values
    Z_list = []
    d_meas_list = []
    for d_arr, disp_arr in zip(all_depth, all_disp):
        valid = (np.isfinite(d_arr) & (d_arr > 0.3) &
                 np.isfinite(disp_arr))
        if valid.any():
            Z_list.append(d_arr[valid].ravel())
            d_meas_list.append(disp_arr[valid].ravel())

    if not Z_list:
        print("  No valid paired depth/disparity pixels!")
        return 1

    Z = np.concatenate(Z_list)
    d_meas = np.concatenate(d_meas_list)

    # ZED disparity sign convention
    print(f"\n  Sign of measured disparity:")
    print(f"    positive pixels: {int((d_meas > 0).sum())}")
    print(f"    negative pixels: {int((d_meas < 0).sum())}")
    print(f"    zero pixels:     {int((d_meas == 0).sum())}")

    # Use absolute value for comparison (ZED may use negative convention)
    d_abs = np.abs(d_meas)

    # For valid range, sample up to 10000 points
    mask = Z > 0.3
    if mask.sum() > 10000:
        idx = np.random.default_rng(42).choice(mask.sum(), 10000, replace=False)
        Z_sample = Z[mask][idx]
        d_sample = d_abs[mask][idx]
    else:
        Z_sample = Z[mask]
        d_sample = d_abs[mask]

    d_expected = fxB / Z_sample

    mae = np.mean(np.abs(d_sample - d_expected))
    med_ae = np.median(np.abs(d_sample - d_expected))
    rmse = np.sqrt(np.mean((d_sample - d_expected) ** 2))

    # Correlation
    if len(Z_sample) > 1:
        corr = np.corrcoef(d_sample, d_expected)[0, 1]
    else:
        corr = float("nan")

    print(f"\n  Sampled {len(Z_sample)} pixels (depth > 0.3m)")
    print(f"  MAE             : {mae:.6f}")
    print(f"  Median AE       : {med_ae:.6f}")
    print(f"  RMSE            : {rmse:.6f}")
    print(f"  Correlation     : {corr:.6f}")

    # Check if using absolute value makes relationship work
    print(f"\n  If disparity sign is inverted (use abs()):")
    print(f"    Expected range at Z=1m: d={fxB/1.0:.4f}")
    print(f"    Expected range at Z=5m: d={fxB/5.0:.4f}")
    print(f"    Measured min (abs): {float(d_abs[d_abs > 0].min()):.6f}" if (d_abs > 0).any() else "    No positive disparity")
    print(f"    Measured max (abs): {float(d_abs.max()):.6f}")

    # ── Save results ───────────────────────────────────────────
    with open("diagnostics/disparity_raw_stats.txt", "w") as f:
        f.write("ZED DISPARITY RAW STATISTICS\n")
        f.write(f"Camera: {info.camera_model} S/N {info.serial_number}\n")
        f.write(f"fx={fx:.4f}  baseline={baseline:.6f}  f*B={fxB:.6f}\n")
        f.write(f"Frames analyzed: {len(all_disp)}\n\n")
        f.write(f"Total pixels: {disp_stack.size}\n")
        f.write(f"Finite: {int(np.isfinite(disp_stack).sum())}\n")
        f.write(f"NaN: {int(np.isnan(disp_stack).sum())}\n")
        f.write(f"Inf: {int(np.isinf(disp_stack).sum())}\n")
        f.write(f"Zero: {int((disp_stack == 0).sum())}\n")
        f.write(f"Negative: {int((disp_stack < 0).sum())}\n")
        f.write(f"Positive: {int((disp_stack > 0).sum())}\n")
        finite_d = disp_stack[np.isfinite(disp_stack)]
        if finite_d.size > 0:
            f.write(f"Min (finite): {float(finite_d.min()):.6f}\n")
            f.write(f"Max (finite): {float(finite_d.max()):.6f}\n")
            f.write(f"Mean (finite): {float(finite_d.mean()):.6f}\n")
            f.write(f"Median (finite): {float(np.median(finite_d)):.6f}\n")
            pcts = [1, 5, 95, 99]
            pvals = np.percentile(finite_d, pcts)
            for p, v in zip(pcts, pvals):
                f.write(f"P{p}: {v:.6f}\n")
        f.write(f"\nSign: positive={int((disp_stack > 0).sum())}  negative={int((disp_stack < 0).sum())}  zero={int((disp_stack == 0).sum())}\n")
        f.write(f"\nDepth-Disparity validation:\n")
        f.write(f"  MAE: {mae:.6f}\n")
        f.write(f"  Median AE: {med_ae:.6f}\n")
        f.write(f"  RMSE: {rmse:.6f}\n")
        f.write(f"  Correlation: {corr:.6f}\n")

    with open("diagnostics/depth_disparity_validation.txt", "w") as f:
        f.write("DEPTH ↔ DISPARITY MATHEMATICAL VALIDATION\n")
        f.write(f"Formula: d = f*B/Z  (f={fx:.4f}, B={baseline:.6f})\n")
        f.write(f"f*B = {fxB:.6f}\n")
        f.write(f"Sampled pixels: {len(Z_sample)}\n")
        f.write(f"MAE: {mae:.6f}\n")
        f.write(f"Median AE: {med_ae:.6f}\n")
        f.write(f"RMSE: {rmse:.6f}\n")
        f.write(f"Correlation: {corr:.6f}\n")
        f.write(f"Sign: positive={int((d_meas > 0).sum())}  negative={int((d_meas < 0).sum())}\n")

    print(f"\nSaved: diagnostics/disparity_raw_stats.txt")
    print(f"Saved: diagnostics/depth_disparity_validation.txt")
    return 0


def _print_frame_stats(idx, depth, disp):
    print(f"\n--- Frame {idx} ---")
    _print_full_stats("disparity", disp.ravel())
    _print_full_stats("depth", depth.ravel())


def _print_full_stats(name, data):
    total = data.size
    finite = np.isfinite(data)
    n_finite = int(finite.sum())
    n_nan = int(np.isnan(data).sum())
    n_inf = int(np.isinf(data).sum())
    n_zero = int((data == 0).sum())
    n_neg = int((data < 0).sum())
    n_pos = int((data > 0).sum())

    print(f"  {name}:")
    print(f"    shape={data.shape}  dtype={data.dtype}")
    print(f"    total={total}  finite={n_finite}  NaN={n_nan}  Inf={n_inf}")
    print(f"    zero={n_zero}  negative={n_neg}  positive={n_pos}")

    vals = data[finite]
    if vals.size > 0:
        print(f"    min={float(vals.min()):.6f}  max={float(vals.max()):.6f}")
        print(f"    mean={float(vals.mean()):.6f}  median={float(np.median(vals)):.6f}")
        pcts = [1, 5, 95, 99]
        pvals = np.percentile(vals, pcts)
        pct_str = "  ".join(f"P{p}={v:.6f}" for p, v in zip(pcts, pvals))
        print(f"    {pct_str}")


if __name__ == "__main__":
    sys.exit(main())
