from dataclasses import dataclass, field
from typing import List

import numpy as np

from dataset.frame import Frame
from mti8.frame import MTiFrame
from radar.frame import RadarFrame, RadarPoint


@dataclass
class GNSSData:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    fix_quality: int = 0
    fix_type: str = "No Fix"
    satellites: int = 0
    hdop: float = 0.0
    speed_mps: float = 0.0
    course_deg: float = 0.0
    rtk_fixed: bool = False
    timestamp_ns: int = 0
    utc_time: str = ""
    gps_week: int = 0
    gps_tow: float = 0.0


@dataclass
class RadarData:
    points: List[RadarPoint] = field(default_factory=list)
    range_doppler: np.ndarray | None = None
    range_azimuth: np.ndarray | None = None
    range_profile: np.ndarray | None = None
    timestamp_ns: int = 0
    frame_id: int = 0
    num_objects: int = 0
    num_range_bins: int = 0
    num_doppler_bins: int = 0


@dataclass
class LidarData:
    points_full: np.ndarray | None = None
    reflectivity: np.ndarray | None = None
    timestamp_ns: int = 0
    frame_id: int = 0
    num_points: int = 0
    complete: bool = False


@dataclass
class SyncedFrame:
    camera: Frame
    imu: MTiFrame
    camera_timestamp_ns: int
    imu_timestamp_ns: int
    delta_ns: int
    gnss: GNSSData = field(default_factory=GNSSData)
    gnss_timestamp_ns: int = 0
    radar: RadarData = field(default_factory=RadarData)
    radar_timestamp_ns: int = 0
    lidar: LidarData = field(default_factory=LidarData)
    lidar_timestamp_ns: int = 0
    frame_uuid: str = ""
