from dataclasses import dataclass, field
import numpy as np


@dataclass
class Frame:
    frame_id: int
    # Monotonic timestamp is retained for camera/IMU synchronization.
    timestamp_ns: int
    left: np.ndarray
    right: np.ndarray
    depth: np.ndarray
    zed_imu: dict = field(default_factory=dict)
    disparity: np.ndarray = None
    confidence: np.ndarray = None
    view_depth: np.ndarray = None
    view_confidence: np.ndarray = None
    exposure: float = 0.0
    gain: float = 0.0
    temperature: float = 0.0
    # UTC timestamp is used for file names and dataset partitions.
    wall_timestamp_ns: int = 0
