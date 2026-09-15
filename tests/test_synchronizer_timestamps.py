"""Synchronizer timestamps must share a unified monotonic time base.

All sensors arrive at the layer above stamped with time.monotonic_ns(), and
the synchronizer pairs every modality to the camera frame on that base.  This
locks in the invariant that `*_timestamp_ns` fields are never 0 when a live
source was matched, and that mismatched/late sources come back empty.
"""

from types import SimpleNamespace

from fusion.synchronizer import Synchronizer
from fusion.synced_frame import SyncedFrame


def _frame(ns):
    return SimpleNamespace(timestamp_ns=ns)


class MTI:
    def __init__(self, ns):
        self._ns = ns

    def get_nearest(self, target):
        return SimpleNamespace(timestamp_ns=self._ns)


class GNSS:
    def __init__(self, ts_ns):
        self._ts = ts_ns

    def latest(self):
        return {
            "lat": 1.0, "lon": 2.0, "alt": 3.0,
            "fix": "RTK", "fix_quality": 4,
            "satellites": 10, "hdop": 1.0,
            "speed": 1.5, "course": 90.0,
            "rtk_fixed": True,
            "timestamp_ns": self._ts,
        }


class RADAR:
    def __init__(self, ns):
        self._ns = ns

    def get_nearest(self, target):
        return SimpleNamespace(
            points=[], range_doppler=None, range_profile=None,
            timestamp_ns=self._ns, frame_id=1,
            num_objects=0, num_range_bins=256, num_doppler_bins=8,
        )


class LIDAR:
    def __init__(self, ns):
        self._ns = ns

    def get_nearest(self, target):
        return SimpleNamespace(
            points_full=None, reflectivity=None,
            timestamp_ns=self._ns, frame_id=2,
            num_points=0, complete=True,
        )


def test_all_sources_share_unified_timestamp_base():
    """Every matched source timestamp equals the camera's monotonic base."""
    cam_ns = 1_700_000_000_000_000_000
    mti = MTI(cam_ns - 1_000_000)      # 1 ms before camera
    gnss = GNSS(cam_ns - 500_000_000)  # ~0.5 s stale GNSS fix (1-5 Hz sensor)
    radar = RADAR(cam_ns + 10_000_000)
    lidar = LIDAR(cam_ns + 40_000_000)

    synced = Synchronizer(mti, gnss_receiver=gnss,
                          radar_receiver=radar,
                          lidar_receiver=lidar).sync(_frame(cam_ns))

    assert synced is not None
    assert synced.camera_timestamp_ns == cam_ns
    # MTi tightly coupled to camera on the same monotonic base.
    assert abs(synced.imu_timestamp_ns - cam_ns) == 1_000_000
    assert synced.delta_ns == 1_000_000
    # GNSS within its slow-update tolerance; timestamp must be populated.
    assert synced.gnss.timestamp_ns == cam_ns - 500_000_000
    assert synced.gnss_timestamp_ns == cam_ns - 500_000_000
    # Radar/Lidar within their gates.
    assert synced.radar_timestamp_ns == cam_ns + 10_000_000
    assert synced.lidar_timestamp_ns == cam_ns + 40_000_000


def test_late_source_returns_empty_not_wrong_timestamp():
    """A source outside its tolerance must not be silently misattributed."""
    cam_ns = 1_700_000_000_000_000_000
    mti = MTI(cam_ns)
    gnss = GNSS(cam_ns)
    radar = RADAR(cam_ns + 400_000_000)   # far beyond RADAR_MAX_DELTA_NS
    lidar = LIDAR(cam_ns + 200_000_000)   # far beyond LIDAR_MAX_DELTA_NS

    synced = Synchronizer(mti, gnss_receiver=gnss,
                          radar_receiver=radar,
                          lidar_receiver=lidar).sync(_frame(cam_ns))

    assert synced is not None
    assert synced.radar_timestamp_ns == 0
    assert synced.lidar_timestamp_ns == 0
    assert synced.radar.num_objects == 0
    assert synced.lidar.num_points == 0
