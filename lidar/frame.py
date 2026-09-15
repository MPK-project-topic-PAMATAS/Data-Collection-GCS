"""Lidar frame dataclasses shared by the receiver, synchronizer and writer."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class LidarFrame:
    frame_id: int = 0
    timestamp_ns: int = 0
    # Downsampled (for viz) and full-resolution XYZ point clouds in metres.
    # Both are Nx3 float arrays; `points` is the small downsampled one used
    # by the dashboard, `points_full` is written to JSON per synchronized
    # camera frame when present.
    points: np.ndarray | None = None
    points_full: np.ndarray | None = None
    reflectivity: np.ndarray | None = None
    num_points: int = 0
    complete: bool = False
