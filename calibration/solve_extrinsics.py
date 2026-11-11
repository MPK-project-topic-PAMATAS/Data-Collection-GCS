"""Solve a sensor's extrinsic pose from correspondences and (optionally)
write it into calibration/extrinsics/extrinsics.yaml.

  lidar-camera  --data corr.npz   arrays: pts_lidar (N,3), pix (N,2)
                                  (cutout-board hole centres / corners, many
                                  board poses; N >= 6)
  radar-lidar   --data corr.npz   arrays: pts_radar (N,3), pts_lidar (N,3)
                                  (corner-reflector positions, N >= 4)

The solved sensor's pose is searched in a box around its CURRENT yaml value
(the tape measurement), all other sensors stay fixed - so measure the fixed
one first and set `measured: true` is NOT required, but the reference sensor
(camera for lidar-camera, lidar for radar-lidar) must already be right.

Examples:
  python calibration/solve_extrinsics.py lidar-camera --data board.npz
  python calibration/solve_extrinsics.py radar-lidar --data refl.npz \
      --range 0.2 0.2 0 0 0 8 --write
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import extrinsic_solvers as xs  # noqa: E402
from fusion.transforms import DEFAULT_PATH, load_extrinsics, save_sensor_pose  # noqa: E402


def _intrinsics_matrix(path: Path) -> np.ndarray:
    import yaml
    d = yaml.safe_load(path.read_text())["rectified"]["left"]
    return np.array([[d["fx"], 0, d["cx"]], [0, d["fy"], d["cy"]], [0, 0, 1.0]])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pair", choices=["lidar-camera", "radar-lidar"])
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--extrinsics", type=Path, default=DEFAULT_PATH)
    ap.add_argument("--intrinsics", type=Path,
                    default=Path(__file__).resolve().parent / "zed_intrinsics.yaml")
    ap.add_argument("--range", type=float, nargs=6, default=None,
                    metavar=("TX", "TY", "TZ", "ROLL", "PITCH", "YAW"),
                    help="search half-range around the current value (m, m, m, deg, deg, deg)")
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--write", action="store_true",
                    help="write the result into the extrinsics yaml")
    ap.add_argument("--max-spread", type=float, default=0.01,
                    help="refuse --write if translation seed spread exceeds this (m)")
    args = ap.parse_args(argv)

    ext = load_extrinsics(args.extrinsics)
    d = np.load(args.data)
    kw = dict(n_seeds=args.seeds)
    if args.range:
        kw["half_range"] = tuple(args.range)
    if args.pair == "lidar-camera":
        res = xs.solve_lidar_camera_correspondences(
            ext, _intrinsics_matrix(args.intrinsics), d["pts_lidar"], d["pix"], **kw)
    else:
        res = xs.solve_radar_lidar(ext, d["pts_radar"], d["pts_lidar"], **kw)

    print(res.summary())
    trusted = (res.std_seeds[:3].max() <= args.max_spread and not res.at_bound)
    if not trusted:
        print("\nNOT trusted: seeds disagree or a parameter hit its bound.")
    if args.write:
        if not trusted:
            print("refusing to --write an untrusted result", file=sys.stderr)
            return 1
        save_sensor_pose(res.sensor, res.translation_m, res.rpy_deg,
                         path=args.extrinsics, source=f"solve_extrinsics:{args.pair}")
        print("wrote", args.extrinsics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
