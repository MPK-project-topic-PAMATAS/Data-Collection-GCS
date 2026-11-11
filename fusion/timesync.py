"""Inter-sensor time-offset estimation (offline, pure numpy).

Every sensor is stamped with the host clock on arrival (see
docs/SYNCHRONIZATION.md), so each sensor carries its own constant-ish
latency (USB/serial/scan-assembly).  Two signals that measure the same
physical motion (e.g. gyro magnitude from the MTi-8 and from the ZED IMU,
or Doppler ego-speed from the radar and speed from the MTi) let us recover
that relative latency by cross-correlation.

Convention: ``estimate_offset(a, b)`` returns ``offset_s`` such that
``t_b_true = t_b - offset_s`` lines B up with A, i.e. the signal B sampled at
host time t equals A at time t - offset_s.  Positive offset => B is stamped
LATE relative to A (B's host stamps lag the physical event).  To correct B,
subtract ``offset_s`` from its timestamps.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class OffsetEstimate:
    offset_s: float        # B stamped late by this much relative to A
    confidence: float      # peak / second-peak ratio of the correlation (>=1)
    peak_corr: float       # normalised correlation at the peak, in [-1, 1]
    n_overlap: int         # samples of overlap used at the peak
    rate_hz: float         # resampling rate used

    @property
    def reliable(self) -> bool:
        return self.peak_corr > 0.5 and self.confidence > 1.3


def _resample(t: np.ndarray, x: np.ndarray, t0: float, t1: float, rate: float):
    grid = np.arange(t0, t1, 1.0 / rate)
    return grid, np.interp(grid, t, x)


def _highpass(x: np.ndarray, win: int) -> np.ndarray:
    """Remove slow trend (bias/drift) with a moving average."""
    if win < 3 or win >= len(x):
        return x - x.mean()
    k = np.ones(win) / win
    return x - np.convolve(x, k, mode="same")


def estimate_offset(t_a, x_a, t_b, x_b, max_lag_s: float = 0.5,
                    rate_hz: float = 200.0, detrend_s: float = 2.0
                    ) -> OffsetEstimate:
    """Estimate the time offset of B relative to A by normalised
    cross-correlation of two scalar signals (times in seconds).

    Both signals are resampled to one uniform grid and de-trended; the
    correlation peak is refined to sub-sample accuracy with a parabola.
    """
    t_a, x_a = np.asarray(t_a, float), np.asarray(x_a, float)
    t_b, x_b = np.asarray(t_b, float), np.asarray(x_b, float)
    if len(t_a) < 8 or len(t_b) < 8:
        raise ValueError("need at least 8 samples per signal")
    lo = max(t_a.min(), t_b.min())
    hi = min(t_a.max(), t_b.max())
    if hi - lo < 6 * max_lag_s:
        raise ValueError("signals overlap too little for the requested lag")

    grid = np.arange(lo, hi, 1.0 / rate_hz)
    det = int(detrend_s * rate_hz)
    a = _highpass(np.interp(grid, t_a, x_a), det)
    b = _highpass(np.interp(grid, t_b, x_b), det)

    max_k = int(round(max_lag_s * rate_hz))
    lags = np.arange(-max_k, max_k + 1)
    lo_i, hi_i = max_k + det // 2, len(grid) - max_k - det // 2
    if hi_i - lo_i < 8:
        raise ValueError("signals overlap too little for the requested lag")
    a_core = a[lo_i:hi_i] - a[lo_i:hi_i].mean()
    a_norm = np.linalg.norm(a_core)
    corr = np.empty(len(lags))
    for i, k in enumerate(lags):
        # B late by +k samples  <=>  B[n + k] == A[n]
        bc = b[lo_i + k: hi_i + k]
        bc = bc - bc.mean()
        bn = np.linalg.norm(bc)
        corr[i] = float(a_core @ bc) / (a_norm * bn) if a_norm > 0 and bn > 0 else 0.0
    return _finalize(corr, lags, rate_hz, len(a_core))


def _finalize(corr, lags, rate_hz, n):
    i = int(np.argmax(corr))
    frac = 0.0
    if 0 < i < len(corr) - 1:
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        denom = y0 - 2 * y1 + y2
        if denom != 0:
            frac = 0.5 * (y0 - y2) / denom
    peak = corr[i]
    # rival peak: strongest *separate* local maximum (the main peak's own
    # shoulders are not rivals, which matters for smooth signals)
    inner = np.arange(1, len(corr) - 1)
    is_max = (corr[inner] > corr[inner - 1]) & (corr[inner] >= corr[inner + 1])
    rivals = inner[is_max & (inner != i)]
    second = corr[rivals].max() if len(rivals) else 0.0
    conf = float(peak / second) if second > 1e-9 else float("inf")
    offset = (lags[i] + frac) / rate_hz
    return OffsetEstimate(offset_s=float(offset), confidence=conf,
                          peak_corr=float(peak), n_overlap=int(n),
                          rate_hz=float(rate_hz))


def drift_check(t_a, x_a, t_b, x_b, n_windows: int = 4, **kw
                ) -> list[OffsetEstimate]:
    """Estimate the offset independently in ``n_windows`` consecutive time
    windows.  A constant offset gives (nearly) equal values; a trend means
    drift; a large spread means jitter that a single constant cannot fix."""
    t_a, t_b = np.asarray(t_a, float), np.asarray(t_b, float)
    lo = max(t_a.min(), t_b.min())
    hi = min(t_a.max(), t_b.max())
    edges = np.linspace(lo, hi, n_windows + 1)
    out = []
    for w0, w1 in zip(edges[:-1], edges[1:]):
        ma = (t_a >= w0) & (t_a <= w1)
        mb = (t_b >= w0 - 1.0) & (t_b <= w1 + 1.0)
        try:
            out.append(estimate_offset(t_a[ma], np.asarray(x_a)[ma],
                                       t_b[mb], np.asarray(x_b)[mb], **kw))
        except ValueError:
            continue
    return out


# ── signal builders ──────────────────────────────────────────────────

def gyro_magnitude(gyro_xyz: np.ndarray) -> np.ndarray:
    """|omega| — invariant to the (unknown) rotation between the two IMUs,
    which is why it is the signal of choice before extrinsics are known."""
    return np.linalg.norm(np.asarray(gyro_xyz, float), axis=1)


def radar_ego_speed(points_xyv: np.ndarray, min_points: int = 5,
                    inlier_tol: float = 0.3, iters: int = 60,
                    seed: int = 0) -> float | None:
    """Forward ego-speed (m/s, positive when moving forward) from one radar
    frame's static points.

    ``points_xyv`` is (N, 4): x (right), y (forward), z (up), radial
    velocity (m/s, TI convention: negative = approaching).  A static point at
    unit bearing u has radial velocity  v_r = -v_ego . u, so for forward motion
    v_r = -v * (y / r).  RANSAC picks the speed that explains most points
    (moving objects become outliers).
    """
    p = np.asarray(points_xyv, float)
    if p.ndim != 2 or p.shape[0] < min_points:
        return None
    r = np.linalg.norm(p[:, :3], axis=1)
    ok = r > 1e-6
    p, r = p[ok], r[ok]
    if len(p) < min_points:
        return None
    cos_f = p[:, 1] / r                  # forward component of bearing
    use = np.abs(cos_f) > 0.2            # near-lateral points carry no speed info
    cos_f, vr = cos_f[use], p[use, 3]
    if len(vr) < min_points:
        return None
    cand = -vr / cos_f
    rng = np.random.default_rng(seed)
    best, best_n = None, -1
    for idx in rng.integers(0, len(cand), size=min(iters, len(cand) * 4)):
        v = cand[idx]
        inl = np.abs(vr - (-v * cos_f)) < inlier_tol
        if inl.sum() > best_n:
            best_n, best = int(inl.sum()), inl
    if best is None or best_n < min_points:
        return None
    # least-squares speed over inliers: vr = -v * cos_f
    c, y = cos_f[best], vr[best]
    return float(-(c @ y) / (c @ c))
