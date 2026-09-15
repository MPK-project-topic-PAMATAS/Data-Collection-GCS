"""MTi-8 sample validation layer.

Sits between packet acquisition (MTiReceiver) and application state.
Every incoming MTiFrame passes through validate() before it can overwrite
the last known good sample.

Pipeline:
    MTi device → packet acquisition → validate() → last_valid_mti_sample
                                                    ↓
                                          GUI / logger / downstream
"""
import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

from mti8.frame import MTiFrame

log = logging.getLogger(__name__)


class MTiSampleState(Enum):
    VALID = "VALID"
    NO_PACKET = "NO_PACKET"
    INVALID_PACKET = "INVALID_PACKET"
    INVALID_ORIENTATION = "INVALID_ORIENTATION"
    INVALID_SENSOR_DATA = "INVALID_SENSOR_DATA"
    SUSPICIOUS_ROTATION = "SUSPICIOUS_ROTATION"


def _has_nan_inf(values) -> bool:
    """Check if a tuple of floats contains NaN or Inf."""
    if values is None:
        return True
    return any(math.isnan(v) or math.isinf(v) for v in values)


def _quat_norm(q: Tuple[float, float, float, float]) -> float:
    qw, qx, qy, qz = q
    return math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)


def _quat_multiply(a, b):
    """Hamilton product: a * b.  Both in (w,x,y,z) order."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conjugate(q):
    """Conjugate (inverse for unit quaternions).  (w,x,y,z) order."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_angle(q: Tuple[float, float, float, float]) -> float:
    """Rotation angle represented by quaternion q.

    theta = 2 * acos(clamp(|q.w|, -1, 1))
    """
    w = max(-1.0, min(1.0, abs(q[0])))
    return 2.0 * math.acos(w)


@dataclass
class MTiCounters:
    total_packets: int = 0
    valid_packets: int = 0
    invalid_packets: int = 0
    dropped_packets: int = 0
    suspicious_packets: int = 0


class MTiValidator:
    """Validates MTiFrame packets and maintains the last valid sample.

    Thread safety: all mutable state is protected by the caller's lock
    (MTiReceiver._lock).  MTiValidator does not acquire any lock itself.
    """

    def __init__(
        self,
        quat_norm_tol: float = 0.05,
        quat_jump_tol: float = 0.5,
        omega_max_rad_s: float = 10.0,
        dt_nominal: float = 0.005,
        dt_margin: float = 1.5,
    ):
        # Configuration
        self.quat_norm_tol = quat_norm_tol
        self.quat_jump_tol = quat_jump_tol
        self.omega_max_rad_s = omega_max_rad_s
        self.dt_nominal = dt_nominal
        self.dt_margin = dt_margin

        # Last valid sample (persistent across invalid samples)
        self._last_valid: Optional[MTiFrame] = None
        self._last_valid_time: float = 0.0

        # Previous valid for rotation discontinuity check
        self._prev_quat: Optional[Tuple[float, ...]] = None
        self._prev_time: float = 0.0

        # Packet counter tracking
        self._prev_packet_counter: Optional[int] = None

        # Counters
        self.counters = MTiCounters()

    @property
    def last_valid(self) -> Optional[MTiFrame]:
        return self._last_valid

    @property
    def last_valid_time(self) -> float:
        return self._last_valid_time

    def validate(self, frame: Optional[MTiFrame]) -> Tuple[MTiFrame, MTiSampleState]:
        """Validate an incoming MTiFrame.

        Returns (display_frame, state):
          - If frame is valid: returns the frame, state = VALID
          - If frame is invalid: returns last_valid (or a zero frame
            if no valid sample has ever been received), state = reason

        The caller must never update application state with the frame
        directly — always use the returned display_frame.
        """
        self.counters.total_packets += 1

        if frame is None:
            return self._reject(MTiSampleState.NO_PACKET)

        # ── Packet-level validation ──────────────────────────────
        state = self._validate_packet(frame)
        if state is not MTiSampleState.VALID:
            return self._reject(state)

        # ── Sensor data validation ───────────────────────────────
        state = self._validate_sensor_data(frame)
        if state is not MTiSampleState.VALID:
            return self._reject(state)

        # ── Orientation validation ───────────────────────────────
        state = self._validate_orientation(frame)
        if state is not MTiSampleState.VALID:
            return self._reject(state)

        # ── Rotation discontinuity check ─────────────────────────
        state = self._check_rotation(frame)
        if state is not None:
            # Log but accept — only mark suspicious, do not reject
            self.counters.suspicious_packets += 1
            log.warning(
                "[MTI] SUSPICIOUS_ROTATION reason=%.3f_rad seq=%d",
                _quat_angle(self._delta_quat) if self._delta_quat else 0.0,
                frame.packet_counter,
            )

        # ── Accept ───────────────────────────────────────────────
        self.counters.valid_packets += 1
        self._last_valid = frame
        self._last_valid_time = time.monotonic()

        # Update rotation tracking
        if frame.quaternion is not None and len(frame.quaternion) >= 4:
            self._prev_quat = tuple(frame.quaternion)
            self._prev_time = frame.timestamp_ns / 1e9 if frame.timestamp_ns > 1e9 else time.monotonic()

        # Update packet counter tracking
        if frame.packet_counter > 0:
            if self._prev_packet_counter is not None:
                gap = frame.packet_counter - self._prev_packet_counter - 1
                if gap > 0:
                    self.counters.dropped_packets += gap
                    log.warning(
                        "[MTI] DROPPED_SAMPLES count=%d (seq %d→%d)",
                        gap, self._prev_packet_counter, frame.packet_counter,
                    )
            self._prev_packet_counter = frame.packet_counter

        log.debug(
            "[MTI] VALID_SAMPLE yaw=%.2f seq=%d",
            self._euler_from_quat(frame.quaternion)[2]
            if frame.quaternion else 0.0,
            frame.packet_counter,
        )

        return frame, MTiSampleState.VALID

    def _validate_packet(self, frame: MTiFrame) -> MTiSampleState:
        """Check packet-level completeness."""
        if frame.timestamp_ns == 0 and frame.packet_counter == 0:
            return MTiSampleState.INVALID_PACKET
        return MTiSampleState.VALID

    def _validate_sensor_data(self, frame: MTiFrame) -> MTiSampleState:
        """Check acceleration and gyroscope for NaN/Inf."""
        if frame.acceleration is not None and _has_nan_inf(frame.acceleration):
            return MTiSampleState.INVALID_SENSOR_DATA
        if frame.gyroscope is not None and _has_nan_inf(frame.gyroscope):
            return MTiSampleState.INVALID_SENSOR_DATA
        if frame.magnetometer is not None and _has_nan_inf(frame.magnetometer):
            return MTiSampleState.INVALID_SENSOR_DATA
        return MTiSampleState.VALID

    def _validate_orientation(self, frame: MTiFrame) -> MTiSampleState:
        """Check quaternion and euler for validity."""
        if frame.quaternion is not None:
            if _has_nan_inf(frame.quaternion):
                return MTiSampleState.INVALID_ORIENTATION

            if len(frame.quaternion) < 4:
                return MTiSampleState.INVALID_ORIENTATION

            q = frame.quaternion
            norm = _quat_norm(q)

            # Normalize if close to unit but not exact
            if abs(norm - 1.0) > self.quat_norm_tol:
                return MTiSampleState.INVALID_ORIENTATION

        if frame.euler is not None and _has_nan_inf(frame.euler):
            return MTiSampleState.INVALID_ORIENTATION

        return MTiSampleState.VALID

    def _check_rotation(self, frame: MTiFrame) -> Optional[MTiSampleState]:
        """Check for suspicious rotation discontinuities.

        Returns SUSPICIOUS_ROTATION if the angular change between
        consecutive valid samples exceeds what angular velocity could
        physically produce.  Does NOT reject — only logs.

        Sets self._delta_quat for logging.
        """
        self._delta_quat = None

        if frame.quaternion is None or self._prev_quat is None:
            return None

        if len(frame.quaternion) < 4 or len(self._prev_quat) < 4:
            return None

        q_prev = self._prev_quat
        q_curr = tuple(frame.quaternion)

        # dq = inverse(q_prev) * q_curr
        q_prev_inv = _quat_conjugate(q_prev)
        dq = _quat_multiply(q_prev_inv, q_curr)

        theta = _quat_angle(dq)
        self._delta_quat = dq

        # Physical check: omega_max * dt * margin
        if frame.timestamp_ns > 0 and self._prev_time > 0:
            dt = abs(frame.timestamp_ns - self._prev_time) / 1e9
        else:
            dt = self.dt_nominal

        theta_max = self.omega_max_rad_s * dt * self.dt_margin

        if theta > theta_max:
            return MTiSampleState.SUSPICIOUS_ROTATION

        return None

    def _reject(self, state: MTiSampleState):
        """Handle a rejected sample.  Returns (display_frame, state)."""
        self.counters.invalid_packets += 1

        # Log with previous valid info
        if self._last_valid is not None:
            age_s = time.monotonic() - self._last_valid_time
            log.warning(
                "[MTI] INVALID_SAMPLE reason=%s age=%.3fs seq_prev=%d",
                state.value, age_s,
                self._last_valid.packet_counter,
            )
        else:
            log.warning(
                "[MTI] INVALID_SAMPLE reason=%s (no valid sample yet)",
                state.value,
            )

        # Return last valid sample, or None if never received one
        if self._last_valid is not None:
            return self._last_valid, state
        return _zero_frame(), state

    def _euler_from_quat(self, q) -> Tuple[float, float, float]:
        """XSens (w,x,y,z) quaternion to euler (roll, pitch, yaw) in degrees."""
        if q is None or len(q) < 4:
            return (0.0, 0.0, 0.0)
        w, x, y, z = q
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(sinr, cosr))
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.degrees(math.asin(sinp))
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(siny, cosy))
        return (roll, pitch, yaw)


def _zero_frame() -> MTiFrame:
    """Return an MTiFrame with zero values — used only when no valid
    sample has ever been received (initial state before any packet)."""
    return MTiFrame()
