"""Extrinsic calibration solvers: DE global search + least-squares refinement.

Each solver estimates ONE sensor's pose in the vehicle frame, in exactly the
parametrisation of calibration/extrinsics/extrinsics.yaml
(translation_m + rpy_deg on top of the nominal axis mapping), holding every
other sensor fixed.  Results can therefore be written straight back with
``fusion.transforms.save_sensor_pose``.

Method (same for every sensor pair):
  1. Differential evolution inside tape-measure bounds, run from several
     independent seeds (global stage; copes with flat/non-convex costs).
  2. Levenberg-Marquardt/Nelder-Mead polish from each seed's best point.
  3. Report the best solution, the seed-to-seed spread (a convergence
     check - large spread means the data do not pin the pose down) and,
     for residual-vector problems, a Jacobian covariance (1-sigma).

PSO/DE are global-search tools: they give no covariance and converge slowly
near the optimum, hence stage 2.  Which parameters are observable is a
property of the DATA (target poses / vehicle motion), not the optimiser -
inspect ``std_seeds`` and ``std_jacobian`` before trusting a result.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Sequence

import numpy as np
from scipy.optimize import differential_evolution, least_squares, minimize

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fusion.transforms import (  # noqa: E402
    NOMINAL_ROTATION, Extrinsics, make_transform, rpy_to_matrix,
    transform_points,
)

PARAM_NAMES = ("tx", "ty", "tz", "roll", "pitch", "yaw")


@dataclass
class SolveResult:
    sensor: str
    translation_m: np.ndarray
    rpy_deg: np.ndarray
    cost: float
    rms: float | None                  # sqrt(mean(residual^2)) if residual-based
    n_residuals: int | None
    seeds_params: np.ndarray           # (n_seeds, 6) polished per-seed solutions
    std_seeds: np.ndarray              # (6,) spread across seeds
    std_jacobian: np.ndarray | None    # (6,) 1-sigma from J^T J (None if n/a)
    at_bound: list[str] = field(default_factory=list)

    @property
    def params(self) -> np.ndarray:
        return np.concatenate([self.translation_m, self.rpy_deg])

    def summary(self) -> str:
        lines = [f"{self.sensor}: cost={self.cost:.6g}"
                 + (f" rms={self.rms:.4g}" if self.rms is not None else "")]
        for i, n in enumerate(PARAM_NAMES):
            unit = "m" if i < 3 else "deg"
            sj = ("" if self.std_jacobian is None
                  else f"  jac_1sigma={self.std_jacobian[i]:.4g}")
            lines.append(f"  {n:5s} {self.params[i]:+10.4f} {unit}"
                         f"  seed_std={self.std_seeds[i]:.4g}{sj}")
        if self.at_bound:
            lines.append("  WARNING at search bound (widen it or check data): "
                         + ", ".join(self.at_bound))
        return "\n".join(lines)


def sensor_transform(sensor: str, params: Sequence[float]) -> np.ndarray:
    """T_vehicle_sensor from [tx,ty,tz,roll,pitch,yaw(deg)]."""
    R = rpy_to_matrix(*params[3:6]) @ NOMINAL_ROTATION[sensor]
    return make_transform(R, params[:3])


def solve_pose(residual_fn: Callable[[np.ndarray], np.ndarray], sensor: str,
               center: Sequence[float], half_range: Sequence[float],
               n_seeds: int = 4, maxiter: int = 150, popsize: int = 15,
               seed: int = 0, fixed: Sequence[str] = ()) -> SolveResult:
    """Minimise ``sum(residual_fn(params)**2)`` over a box.

    center/half_range: 6 values each (m, m, m, deg, deg, deg).  Parameters
    named in ``fixed`` are pinned at ``center`` (use for unobservable DOFs,
    e.g. fixed=("tz","roll","pitch") when the data are planar).
    """
    center = np.asarray(center, float)
    half = np.asarray(half_range, float).copy()
    for n in fixed:
        half[PARAM_NAMES.index(n)] = 0.0
    free = half > 0
    lo, hi = center - half, center + half

    def expand(x):
        p = center.copy()
        p[free] = x
        return p

    def resid(x):
        return np.atleast_1d(np.asarray(residual_fn(expand(x)), float))

    def scalar(x):
        r = resid(x)
        return float(r @ r)

    bounds = list(zip(lo[free], hi[free]))
    sols, costs = [], []
    for k in range(n_seeds):
        de = differential_evolution(scalar, bounds, maxiter=maxiter,
                                    popsize=popsize, seed=seed + k, tol=1e-9,
                                    polish=False, init="sobol",
                                    updating="immediate")
        x = de.x
        try:
            ls = least_squares(resid, x, bounds=(lo[free], hi[free]),
                               x_scale=np.maximum(half[free], 1e-9) * 0.1)
            if ls.cost * 2 <= de.fun:
                x = ls.x
        except Exception:
            nm = minimize(scalar, x, method="Nelder-Mead")
            if nm.fun <= de.fun:
                x = nm.x
        sols.append(expand(x))
        costs.append(scalar(x))
    sols = np.array(sols)
    best = int(np.argmin(costs))
    pbest = sols[best]

    std_j = None
    n_res = rms = None
    r = resid(pbest[free])
    if r.size > free.sum():
        n_res = int(r.size)
        rms = float(np.sqrt(np.mean(r ** 2)))
        try:
            J = least_squares(resid, pbest[free], max_nfev=1,
                              bounds=(lo[free], hi[free])).jac
            cov = np.linalg.inv(J.T @ J) * (r @ r) / (r.size - free.sum())
            sj = np.zeros(6)
            sj[free] = np.sqrt(np.clip(np.diag(cov), 0, None))
            std_j = sj
        except np.linalg.LinAlgError:
            std_j = np.full(6, np.inf)
    at_bound = [PARAM_NAMES[i] for i in range(6) if free[i] and
                (abs(pbest[i] - lo[i]) < 1e-3 * half[i]
                 or abs(pbest[i] - hi[i]) < 1e-3 * half[i])]
    return SolveResult(sensor, pbest[:3], pbest[3:], float(costs[best]), rms,
                       n_res, sols, sols.std(axis=0), std_j, at_bound)


def _start(ext: Extrinsics, sensor: str):
    p = ext.params[sensor]
    return np.array(list(p["translation_m"]) + list(p["rpy_deg"]), float)


# ── lidar <-> camera ─────────────────────────────────────────────────

def project_to_pixels(K: np.ndarray, pts_cam: np.ndarray):
    """Pinhole projection of camera-frame points (x right, y down, z fwd).
    Returns (uv (N,2), in_front mask)."""
    z = pts_cam[:, 2]
    ok = z > 0.1
    zs = np.where(ok, z, 1.0)
    u = K[0, 0] * pts_cam[:, 0] / zs + K[0, 2]
    v = K[1, 1] * pts_cam[:, 1] / zs + K[1, 2]
    return np.c_[u, v], ok


def solve_lidar_camera_correspondences(
        ext: Extrinsics, K: np.ndarray, pts_lidar: np.ndarray,
        pix: np.ndarray, half_range=(0.15, 0.15, 0.15, 5, 5, 5),
        pix_sigma: float = 1.0, **kw) -> SolveResult:
    """Target-based: 3-D lidar points (e.g. cutout-board hole centres, board
    corners) and their matching image pixels (N >= 6, from several board
    poses).  Optimises the LIDAR pose; the camera pose stays as in ``ext``.
    Residual = reprojection error in pixels / ``pix_sigma``."""
    pts_lidar = np.asarray(pts_lidar, float)
    pix = np.asarray(pix, float)
    if len(pts_lidar) < 6 or len(pts_lidar) != len(pix):
        raise ValueError("need >= 6 matching lidar/pixel pairs")
    T_vc_inv = np.linalg.inv(ext.T_vehicle_sensor("camera"))

    def resid(p):
        T_cl = T_vc_inv @ sensor_transform("lidar", p)
        uv, ok = project_to_pixels(K, transform_points(T_cl, pts_lidar))
        r = (uv - pix) / pix_sigma
        r[~ok] = 1e3                      # behind the camera: heavy penalty
        return r.ravel()

    return solve_pose(resid, "lidar", _start(ext, "lidar"), half_range, **kw)


def nid(a: np.ndarray, b: np.ndarray, bins: int = 32) -> float:
    """Normalised information distance in [0, 1] between two samples
    (0 = one determines the other).  Robust to the lidar-intensity vs
    image-brightness gain/offset mismatch, which is why it is used for
    targetless lidar-camera alignment on road markings."""
    h, _, _ = np.histogram2d(a, b, bins=bins)
    pxy = h / max(h.sum(), 1)
    px, py = pxy.sum(1), pxy.sum(0)
    nz = pxy > 0
    hxy = -(pxy[nz] * np.log(pxy[nz])).sum()
    hx = -(px[px > 0] * np.log(px[px > 0])).sum()
    hy = -(py[py > 0] * np.log(py[py > 0])).sum()
    mi = hx + hy - hxy
    return float((hxy - mi) / hxy) if hxy > 0 else 1.0


def solve_lidar_camera_nid(
        ext: Extrinsics, K: np.ndarray,
        scans: Sequence[tuple[np.ndarray, np.ndarray, np.ndarray]],
        half_range=(0.10, 0.10, 0.10, 3, 3, 3), bins: int = 32,
        max_points: int = 4000, **kw) -> SolveResult:
    """Targetless: ``scans`` = [(points_lidar (N,3), intensity (N,), gray
    image (H,W)), ...] from city-road drives (lane markings, manhole covers,
    kerbs give strong, repeatable edges).  Minimises the mean NID between
    lidar intensity and image brightness at the projected points.
    Use frames from several scenes and a static vehicle (or de-skewed scans)."""
    prep = []
    rng = np.random.default_rng(0)
    for pts, inten, img in scans:
        pts, inten = np.asarray(pts, float), np.asarray(inten, float)
        if len(pts) > max_points:
            idx = rng.choice(len(pts), max_points, replace=False)
            pts, inten = pts[idx], inten[idx]
        prep.append((pts, inten, np.asarray(img, float)))
    if not prep:
        raise ValueError("no scans given")
    T_vc_inv = np.linalg.inv(ext.T_vehicle_sensor("camera"))

    def cost(p):
        T_cl = T_vc_inv @ sensor_transform("lidar", p)
        total, n = 0.0, 0
        for pts, inten, img in prep:
            uv, ok = project_to_pixels(K, transform_points(T_cl, pts))
            ui, vi = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
            ok &= (ui >= 0) & (ui < img.shape[1]) & (vi >= 0) & (vi < img.shape[0])
            if ok.sum() < 50:
                total += 1.0              # almost nothing overlaps: worst case
            else:
                total += nid(inten[ok], img[vi[ok], ui[ok]], bins)
            n += 1
        return np.array([np.sqrt(total / n)])   # sqrt so r@r == mean NID

    return solve_pose(cost, "lidar", _start(ext, "lidar"), half_range, **kw)


# ── radar <-> lidar ──────────────────────────────────────────────────

def solve_radar_lidar(
        ext: Extrinsics, pts_radar: np.ndarray, pts_lidar: np.ndarray,
        half_range=(0.15, 0.15, 0.15, 5, 5, 5), range_sigma: float = 0.10,
        az_sigma_deg: float = 2.0, el_sigma_deg: float | None = None, **kw
        ) -> SolveResult:
    """Corner-reflector based: matching reflector positions seen by the radar
    (``pts_radar``, radar frame, x right / y fwd / z up) and by the lidar
    (``pts_lidar``, lidar frame).  Optimises the RADAR pose; the lidar pose
    stays as in ``ext``.

    The AWR1843 has coarse azimuth (~15 deg resolution) and poor elevation,
    so residuals are formed in radar polar space and weighted by the radar's
    own accuracy instead of in Cartesian metres.  Elevation is ignored unless
    ``el_sigma_deg`` is given; consequently also pass ``fixed=("tz", "roll",
    "pitch")`` unless reflectors are placed at several heights."""
    pts_radar = np.asarray(pts_radar, float)
    pts_lidar = np.asarray(pts_lidar, float)
    if len(pts_radar) < 4 or len(pts_radar) != len(pts_lidar):
        raise ValueError("need >= 4 matching radar/lidar reflector positions")
    T_vl = ext.T_vehicle_sensor("lidar")

    def polar(p):
        r = np.linalg.norm(p, axis=1)
        az = np.degrees(np.arctan2(p[:, 0], p[:, 1]))          # 0 = boresight (+y)
        el = np.degrees(np.arcsin(np.clip(p[:, 2] / np.maximum(r, 1e-9), -1, 1)))
        return r, az, el

    r_m, az_m, el_m = polar(pts_radar)

    def resid(p):
        T_rl = np.linalg.inv(sensor_transform("radar", p)) @ T_vl
        r, az, el = polar(transform_points(T_rl, pts_lidar))
        out = [(r - r_m) / range_sigma, (az - az_m) / az_sigma_deg]
        if el_sigma_deg:
            out.append((el - el_m) / el_sigma_deg)
        return np.concatenate(out)

    kw.setdefault("fixed", ("tz", "roll", "pitch") if not el_sigma_deg else ())
    return solve_pose(resid, "radar", _start(ext, "radar"), half_range, **kw)


def solve_radar_yaw_from_doppler(speeds_vr: Sequence[np.ndarray],
                                 bearings_xy: Sequence[np.ndarray],
                                 v_ref: Sequence[float]) -> float:
    """Radar mounting yaw (deg) from ego-motion: for static points
    v_r = -v_ref * cos(bearing_in_vehicle).  Given per-frame radial
    velocities, radar-frame bearings (x,y unit-ish) and the reference forward
    speed (lidar odometry / MTi), find the yaw that makes them consistent.
    Complements the reflector solve when targets cannot span azimuth."""
    from scipy.optimize import minimize_scalar

    def cost(yaw_deg):
        c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        err = 0.0
        for vr, b, v in zip(speeds_vr, bearings_xy, v_ref):
            b = np.asarray(b, float)
            n = np.linalg.norm(b, axis=1, keepdims=True)
            u = b / np.maximum(n, 1e-9)
            # vehicle-forward component of the bearing for a radar mounted
            # with yaw psi: x_veh = cos(psi)*y_r + sin(psi)*x_r
            fx = u[:, 1] * c + u[:, 0] * s
            err += np.median(np.abs(np.asarray(vr) + v * fx))
        return err

    res = minimize_scalar(cost, bounds=(-30, 30), method="bounded")
    return float(res.x)
