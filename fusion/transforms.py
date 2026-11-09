"""Rigid-body geometry for the sensor rig.

Vehicle frame: origin at the MTi-8 IMU center, x forward, y left, z up
(ROS REP-103).  Every sensor is described by T_vehicle_<sensor>, the pose of
the sensor frame expressed in the vehicle frame, so that

    p_vehicle = T_vehicle_sensor @ p_sensor          (4x4 homogeneous)

Sensor-native axes (the "nominal" rotation, before any mount tilt):
    imu     x fwd,   y left,  z up      (identity)
    lidar   x fwd,   y left,  z up      (Ouster sensor frame, identity)
    radar   x right, y fwd,   z up      (AWR1843 TLV frame, radar/parser.py)
    camera  x right, y down,  z fwd     (ZED optical frame, left eye)
    gnss    antenna phase center, orientation unused

Poses live in calibration/extrinsics/extrinsics.yaml: a translation in
metres (vehicle axes) and an optional ``rpy_deg`` mount rotation
(roll/pitch/yaw about the vehicle x/y/z axes, extrinsic, applied on top of
the nominal axis mapping).  Entries with ``measured: false`` are
placeholders; ``load_extrinsics(strict=True)`` refuses them.
"""
from __future__ import annotations

import warnings
from pathlib import Path

import numpy as np

DEFAULT_PATH = (Path(__file__).resolve().parent.parent
                / "calibration" / "extrinsics" / "extrinsics.yaml")

# R_vehicle_sensor for a sensor mounted square to the vehicle.
NOMINAL_ROTATION = {
    "imu": np.eye(3),
    "lidar": np.eye(3),
    "gnss": np.eye(3),
    # radar x=right,y=fwd,z=up -> vehicle (x=fwd, y=left, z=up)
    "radar": np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float),
    # camera x=right,y=down,z=fwd -> vehicle
    "camera": np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float),
}


def rpy_to_matrix(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0) -> np.ndarray:
    """Rotation R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_transform(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float)
    return T


def invert(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_transform(R.T, -R.T @ t)


def transform_points(T: np.ndarray, pts) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array (or a single 3-vector)."""
    a = np.atleast_2d(np.asarray(pts, float))
    out = a @ T[:3, :3].T + T[:3, 3]
    return out if np.ndim(pts) > 1 else out[0]


class Extrinsics:
    """All sensor poses in the vehicle frame, with chaining helpers."""

    def __init__(self, poses: dict[str, np.ndarray], measured: dict[str, bool],
                 params: dict[str, dict] | None = None):
        self._T = poses          # name -> T_vehicle_sensor
        self.measured = measured
        # raw yaml values (translation_m, rpy_deg) so UIs can edit/save them
        self.params = params or {}

    def set_pose(self, sensor: str, translation_m, rpy_deg=(0.0, 0.0, 0.0)):
        """Replace a sensor's pose in memory (same meaning as the yaml)."""
        R = rpy_to_matrix(*rpy_deg) @ NOMINAL_ROTATION[sensor]
        self._T[sensor] = make_transform(R, translation_m)
        self.params[sensor] = {"translation_m": [float(v) for v in translation_m],
                               "rpy_deg": [float(v) for v in rpy_deg]}

    @property
    def sensors(self):
        return sorted(self._T)

    def T_vehicle_sensor(self, sensor: str) -> np.ndarray:
        return self._T[sensor].copy()

    def T(self, target: str, source: str) -> np.ndarray:
        """T_target_source: maps points in the source frame into the target
        frame.  'vehicle' is accepted as either name."""
        def pose(n):
            return np.eye(4) if n == "vehicle" else self._T[n]
        return invert(pose(target)) @ pose(source)

    def transform(self, pts, source: str, target: str) -> np.ndarray:
        return transform_points(self.T(target, source), pts)

    def radar_to_camera_offset(self) -> tuple[float, float, float]:
        """Legacy lever arm: camera origin minus radar origin, expressed in
        the radar's own axes (x=right, y=fwd, z=up) — the convention of
        config.settings.RADAR_TO_CAMERA_OFFSET_M.  Valid only when the two
        sensors are boresight-parallel."""
        t_rc = self.T("radar", "camera")[:3, 3]
        return tuple(float(v) for v in t_rc)


def load_extrinsics(path=DEFAULT_PATH, strict: bool = False) -> Extrinsics:
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    poses, measured, params = {}, {}, {}
    for name, e in doc["sensors"].items():
        if name not in NOMINAL_ROTATION:
            raise ValueError(f"unknown sensor '{name}' in {path}")
        rpy = e.get("rpy_deg", [0.0, 0.0, 0.0])
        R = rpy_to_matrix(*rpy) @ NOMINAL_ROTATION[name]
        poses[name] = make_transform(R, e["translation_m"])
        measured[name] = bool(e.get("measured", False))
        params[name] = {"translation_m": [float(v) for v in e["translation_m"]],
                        "rpy_deg": [float(v) for v in rpy]}
    missing = [n for n, m in measured.items() if not m and n != "imu"]
    if missing:
        msg = ("extrinsics not measured (placeholder values): "
               + ", ".join(sorted(missing)))
        if strict:
            raise ValueError(msg)
        warnings.warn(msg, stacklevel=2)
    return Extrinsics(poses, measured, params)


def save_sensor_pose(sensor: str, translation_m, rpy_deg, path=DEFAULT_PATH,
                     source: str = "visual_alignment") -> None:
    """Rewrite one sensor's block in the extrinsics yaml (other blocks and
    comments are left untouched) and mark it measured."""
    import re
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    tx = ", ".join(f"{float(v):.4f}" for v in translation_m)
    rp = ", ".join(f"{float(v):.2f}" for v in rpy_deg)
    block = (f"  {sensor}:\n"
             f"    translation_m: [{tx}]\n"
             f"    rpy_deg: [{rp}]\n"
             f"    measured: true\n"
             f"    source: {source}\n")
    pat = re.compile(rf"^  {re.escape(sensor)}:.*?(?=^  \w+:|\Z)", re.S | re.M)
    if not pat.search(text):
        raise ValueError(f"sensor '{sensor}' not found in {path}")
    path.write_text(pat.sub(lambda m: block, text, count=1), encoding="utf-8")
