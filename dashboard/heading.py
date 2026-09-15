"""GNSS-course yaw drift correction for the MTi-8 dashboard display.

The MTi-8 heading is gyro-integrated and has no absolute yaw reference, so it
drifts over time and carries any startup mount offset. When the vehicle moves
fast enough, GNSS course-over-ground is a good heading reference: this
corrector slowly estimates a yaw bias from (gnss_course - mti_yaw) and applies
it always. Recording is unaffected — raw MTi data is written unmodified.
"""

from __future__ import annotations


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class GNSSYawCorrector:
    """Integrate a yaw bias toward GNSS course while the vehicle moves."""

    def __init__(self, gain: float = 0.05, min_speed: float = 1.5):
        self.gain = gain          # bias convergence: deg/s per deg of error
        self.min_speed = min_speed
        self._bias = 0.0
        self._last_t = None

    def correct(self, yaw_deg: float, course_deg: float,
                speed_mps: float, t: float) -> float:
        """Return yaw with the accumulated GNSS-based bias applied."""
        if speed_mps >= self.min_speed:
            dt = (t - self._last_t) if self._last_t is not None else 0.0
            self._last_t = t
            dt = max(0.0, min(dt, 1.0))
            err = _wrap180(course_deg - (yaw_deg + self._bias))
            self._bias += self.gain * err * dt
        else:
            self._last_t = None  # restart cleanly on the next motion
        return _wrap180(yaw_deg + self._bias)


def _demo():
    """Self-check: stationary passes through, moving converges to course."""
    c = GNSSYawCorrector(gain=0.1, min_speed=2.0)

    y = c.correct(100.0, 200.0, 1.0, 0.0)   # stationary: bias frozen
    assert y == 100.0
    y = c.correct(100.0, 200.0, 1.0, 0.5)   # still stationary
    assert y == 100.0

    yaw = 100.0                             # true heading is 90°, IMU reads 100°
    for i in range(1, 401):
        yaw = c.correct(yaw, 90.0, 5.0, i * 0.1)
    assert abs(yaw - 90.0) < 2.0, yaw

    # Yaw stays wrapped to [-180, 180)
    assert -180.0 <= yaw < 180.0
    print("heading: OK")


if __name__ == "__main__":
    _demo()
