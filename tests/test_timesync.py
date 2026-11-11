import numpy as np
import pytest

from fusion.timesync import (
    drift_check, estimate_offset, gyro_magnitude, radar_ego_speed,
)


def _signal(t):
    rng = np.random.default_rng(1)
    knots = np.cumsum(rng.normal(size=400))
    return np.interp(t, np.linspace(t.min(), t.max(), 400), knots)


def test_recovers_known_offset_subsample():
    ta = np.arange(0, 60, 0.005)                 # 200 Hz
    tb = np.arange(0, 60, 1 / 30.0)              # 30 Hz, different rate
    true = 0.0437                                # B stamped 43.7 ms late
    xa = _signal(ta)
    xb = np.interp(tb - true, ta, xa)            # B(t) = A(t - true)
    est = estimate_offset(ta, xa, tb, xb, max_lag_s=0.3)
    assert est.offset_s == pytest.approx(true, abs=0.002)
    assert est.reliable


def test_negative_offset_and_noise():
    ta = np.arange(0, 80, 0.01)
    tb = np.arange(0, 80, 0.01)
    xa = _signal(ta)
    xb = np.interp(tb + 0.12, ta, xa) + np.random.default_rng(2).normal(0, 0.05, len(tb))
    est = estimate_offset(ta, xa, tb, xb, max_lag_s=0.4)
    assert est.offset_s == pytest.approx(-0.12, abs=0.003)


def test_uncorrelated_signals_not_reliable():
    ta = np.arange(0, 60, 0.01)
    xa = np.random.default_rng(3).normal(size=len(ta))
    xb = np.random.default_rng(4).normal(size=len(ta))
    assert not estimate_offset(ta, xa, ta, xb).reliable


def test_drift_check_detects_trend():
    ta = np.arange(0, 120, 0.01)
    xa = _signal(ta)
    # offset grows linearly 0.02 -> 0.06 s (clock drift)
    off = 0.02 + 0.04 * ta / 120
    xb = np.interp(ta - off, ta, xa)
    ests = drift_check(ta, xa, ta, xb, n_windows=4, max_lag_s=0.2)
    offs = [e.offset_s for e in ests]
    assert len(offs) == 4 and offs == sorted(offs)
    assert offs[-1] - offs[0] > 0.02


def test_too_short_raises():
    t = np.arange(0, 1, 0.01)
    with pytest.raises(ValueError):
        estimate_offset(t, t, t, t, max_lag_s=0.5)


def test_gyro_magnitude():
    assert np.allclose(gyro_magnitude([[3, 4, 0], [0, 0, 2]]), [5, 2])


def test_radar_ego_speed_with_moving_outliers():
    rng = np.random.default_rng(5)
    v = 8.0
    ang = rng.uniform(-1.0, 1.0, 60)
    r = rng.uniform(5, 40, 60)
    x, y = r * np.sin(ang), r * np.cos(ang)
    vr = -v * (y / r)
    vr[:6] = rng.uniform(-3, 3, 6) + 5           # moving objects
    pts = np.c_[x, y, np.zeros(60), vr]
    assert radar_ego_speed(pts) == pytest.approx(v, abs=0.1)
    assert radar_ego_speed(pts[:3]) is None
