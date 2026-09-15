"""Validate exported ZED calibration against SDK depth.

Loads zed_intrinsics.yaml, grabs one stereo frame, and compares depth
computed from the exported calibration (Z = fx * B / d) against the
ZED SDK's own depth map.

Usage:
    python calibration/validate_zed_calibration.py
    python calibration/validate_zed_calibration.py --calibration calibration/zed_intrinsics.yaml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np
import yaml
from pyzed import sl

from config.settings import CAMERA_RESOLUTION, CAMERA_FPS


def load_calibration(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def validate(calib_path: Path, samples: int = 1000):
    calib = load_calibration(calib_path)

    fx = calib["rectified"]["left"]["fx"]
    baseline = calib["baseline_m"]

    print(f"Loaded calibration: {calib['camera_model']} (SN {calib['serial_number']})")
    print(f"  fx={fx:.2f}  baseline={baseline:.4f} m")

    cam = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = CAMERA_RESOLUTION
    init.camera_fps = CAMERA_FPS
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    init.coordinate_units = sl.UNIT.METER

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")

    if cam.grab() != sl.ERROR_CODE.SUCCESS:
        cam.close()
        raise RuntimeError("Failed to grab frame")

    depth_mat = sl.Mat()
    disparity_mat = sl.Mat()
    left_mat = sl.Mat()

    cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
    cam.retrieve_measure(disparity_mat, sl.MEASURE.DISPARITY)
    cam.retrieve_image(left_mat, sl.VIEW.LEFT)
    cam.close()

    depth_sdk = depth_mat.get_data()
    disparity = np.abs(disparity_mat.get_data())
    left = left_mat.get_data()

    valid = (disparity > 0) & np.isfinite(depth_sdk) & (depth_sdk > 0)
    ys, xs = np.where(valid)

    if len(ys) == 0:
        print("No valid pixels found")
        return

    rng = np.random.default_rng(42)
    idx = rng.choice(len(ys), size=min(samples, len(ys)), replace=False)
    ys, xs = ys[idx], xs[idx]

    disp_sampled = disparity[ys, xs]
    depth_sdk_sampled = depth_sdk[ys, xs]
    depth_calc = fx * baseline / disp_sampled

    error = np.abs(depth_calc - depth_sdk_sampled)

    print(f"\nSamples        : {len(ys)}")
    print(f"Mean error     : {error.mean():.4f} m")
    print(f"RMSE           : {np.sqrt((error**2).mean()):.4f} m")
    print(f"Maximum error  : {error.max():.4f} m")
    print(f"Median error   : {np.median(error):.4f} m")

    bias = depth_calc - depth_sdk_sampled
    print(f"Mean bias      : {bias.mean():.4f} m ({'overestimate' if bias.mean() > 0 else 'underestimate'})")

    depth_calc_full = np.full_like(depth_sdk, np.nan)
    depth_calc_full[valid] = fx * baseline / disparity[valid]
    err_map = np.abs(depth_calc_full - depth_sdk)
    err_map[~valid] = np.nan

    err_vis = np.zeros((*depth_sdk.shape, 3), dtype=np.uint8)
    valid_mask = np.isfinite(err_map)
    if valid_mask.any():
        e = err_map[valid_mask]
        e_clipped = np.clip(e / 0.05, 0, 1)
        err_vis[valid_mask, 0] = (e_clipped * 255).astype(np.uint8)
        err_vis[valid_mask, 2] = ((1 - e_clipped) * 255).astype(np.uint8)

    left_bgr = cv2.cvtColor(left, cv2.COLOR_BGRA2BGR) if left.shape[2] == 4 else left
    sidebyside = np.hstack([left_bgr, err_vis])
    out_path = calib_path.parent / "validation_error.png"
    cv2.imwrite(str(out_path), sidebyside)
    print(f"\nError map saved to {out_path}")

    threshold = 0.02
    pass_rate = (error < threshold).mean() * 100
    print(f"Pixels within {threshold*100:.0f} cm: {pass_rate:.1f}%")

    if error.mean() < 0.02 and np.sqrt((error**2).mean()) < 0.03:
        print("\nPASS — calibration is consistent with SDK depth")
    else:
        print("\nFAIL — investigate units or disparity interpretation")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate ZED calibration")
    parser.add_argument(
        "--calibration", "-c",
        type=Path,
        default=Path("calibration/zed_intrinsics.yaml"),
    )
    parser.add_argument("--samples", "-n", type=int, default=1000)
    args = parser.parse_args()
    validate(args.calibration, args.samples)
