from dataclasses import dataclass, field
from typing import List

import numpy as np


@dataclass
class RadarPoint:
    """Single detected point from the AWR1843 point cloud (Cartesian, meters)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    velocity_mps: float = 0.0


@dataclass
class RadarFrame:
    """One decoded radar frame from the AWR1843 data port."""
    points: List[RadarPoint] = field(default_factory=list)
    # (nRange, nDoppler) uint16 power values.  Rows are range bins and
    # columns are Doppler bins, matching the TI SDK 3 output order.
    range_doppler: "np.ndarray | None" = None
    range_profile: "np.ndarray | None" = None   # (nRange,) float32
    timestamp_ns: int = 0
    frame_id: int = 0
    num_objects: int = 0
    num_range_bins: int = 0
    num_doppler_bins: int = 0
