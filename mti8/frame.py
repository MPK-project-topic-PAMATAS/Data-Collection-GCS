from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class MTiFrame:
    """Decoded MTData2 frame. All values straight from the sensor — no
    euler conversion, no convention assumptions.

    XDI 0x2010 quaternion is transmitted as q0, q1, q2, q3 where q0 is the
    real component (w) and q1,q2,q3 are imaginary (x,y,z).
    Wire format: (w, x, y, z) — NOT (x, y, z, w).

    The quaternion qLS represents body->world rotation:
        v_world = R(qLS) @ v_body

    See mti8/quaternion.py for canonical conversion functions.
    """

    # Timestamp group
    packet_counter: int = 0
    sample_time_fine: int = 0
    sample_time_coarse: int = 0

    # Orientation
    quaternion: Optional[Tuple[float, ...]] = None   # (w, x, y, z) float32 x4 — XSens wire order
    euler: Optional[Tuple[float, ...]] = None         # (roll, pitch, yaw) float32 x3
    rotation_matrix: Optional[Tuple[float, ...]] = None  # float32 x9

    # Calibrated inertial
    acceleration: Optional[Tuple[float, ...]] = None  # (x, y, z) m/s²
    gyroscope: Optional[Tuple[float, ...]] = None     # (x, y, z) rad/s
    magnetometer: Optional[Tuple[float, ...]] = None  # (x, y, z) normalized

    # Delta values
    delta_velocity: Optional[Tuple[float, ...]] = None   # (x, y, z) m/s
    delta_quaternion: Optional[Tuple[float, ...]] = None  # (w, x, y, z) a.u.

    # Position / GNSS
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_msl: float = 0.0
    altitude_ellipsoid: float = 0.0

    # Velocity
    velocity: Optional[Tuple[float, ...]] = None  # (x, y, z) m/s

    # Environmental
    temperature: float = 0.0
    baro_pressure: float = 0.0

    # Status
    status_word: int = 0
    status_byte: int = 0

    # Set by receiver (monotonic clock)
    timestamp_ns: int = 0
