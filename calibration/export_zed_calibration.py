"""Export complete ZED stereo calibration to YAML.

Run once per camera configuration. Reads raw (unrectified) and rectified
parameters from the ZED SDK and writes a single immutable calibration file.

Usage:
    python calibration/export_zed_calibration.py
    python calibration/export_zed_calibration.py --output calibration/zed_intrinsics.yaml
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import yaml
from pyzed import sl

from config.settings import CAMERA_RESOLUTION, CAMERA_FPS


def export_calibration(output_path: Path):
    cam = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = CAMERA_RESOLUTION
    init.camera_fps = CAMERA_FPS
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED open failed: {status}")

    info = cam.get_camera_information()
    cp = info.camera_configuration.calibration_parameters
    raw = info.camera_configuration.calibration_parameters_raw

    left = cp.left_cam
    right = cp.right_cam
    raw_left = raw.left_cam
    raw_right = raw.right_cam

    T = cp.stereo_transform.m  # 4x4 numpy, row-major
    R = T[:3, :3].tolist()
    t = T[:3, 3].tolist()
    baseline = float(cp.get_camera_baseline())

    width = info.camera_configuration.resolution.width
    height = info.camera_configuration.resolution.height

    def cam_dict(cam_params, lens_model):
        return {
            "fx": float(cam_params.fx),
            "fy": float(cam_params.fy),
            "cx": float(cam_params.cx),
            "cy": float(cam_params.cy),
            "distortion": [float(d) for d in cam_params.disto],
            "lens_model": str(lens_model),
            "fov": {
                "diagonal_deg": float(cam_params.d_fov),
                "horizontal_deg": float(cam_params.h_fov),
                "vertical_deg": float(cam_params.v_fov),
            },
            "focal_length_mm": float(cam_params.focal_length_metric),
        }

    P_left = [[float(left.fx), 0.0, float(left.cx), 0.0],
              [0.0, float(left.fy), float(left.cy), 0.0],
              [0.0, 0.0, 1.0, 0.0]]

    P_right = [[float(right.fx), 0.0, float(right.cx), -float(right.fx) * baseline],
               [0.0, float(right.fy), float(right.cy), 0.0],
               [0.0, 0.0, 1.0, 0.0]]

    calib = {
        "camera_model": str(info.camera_model),
        "serial_number": str(info.serial_number),
        "resolution": {"width": width, "height": height},
        "fps": info.camera_configuration.fps,
        "baseline_m": baseline,
        "stereo_rotation": R,
        "stereo_translation": t,
        "rectified": {
            "left": cam_dict(left, left.lens_distortion_model),
            "right": cam_dict(right, right.lens_distortion_model),
        },
        "raw": {
            "left": cam_dict(raw_left, raw_left.lens_distortion_model),
            "right": cam_dict(raw_right, raw_right.lens_distortion_model),
        },
        "projection_matrix": {
            "left": P_left,
            "right": P_right,
        },
        "rectification_matrix": {
            "left": np.eye(3).tolist(),
            "right": np.eye(3).tolist(),
        },
    }

    cam.close()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(calib, f, default_flow_style=False, sort_keys=False)

    print(f"Calibration saved to {output_path}")
    print(f"  Camera  : {calib['camera_model']} (SN {calib['serial_number']})")
    print(f"  Resolution: {width}x{height} @ {calib['fps']} FPS")
    print(f"  Baseline : {baseline:.4f} m")
    print(f"  Left fx  : {left.fx:.2f}  cx: {left.cx:.2f}  cy: {left.cy:.2f}")
    print(f"  Right fx : {right.fx:.2f}  cx: {right.cx:.2f}  cy: {right.cy:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export ZED calibration to YAML")
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=Path("calibration/zed_intrinsics.yaml"),
    )
    args = parser.parse_args()
    export_calibration(args.output)
