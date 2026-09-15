from fusion.synced_frame import SyncedFrame, GNSSData, RadarData, LidarData
from mti8.receiver import MTiReceiver
from config.settings import (
    MAX_DELTA_NS, RADAR_MAX_DELTA_NS, LIDAR_MAX_DELTA_NS,
)


class Synchronizer:
    def __init__(self, mti_receiver: MTiReceiver, gnss_receiver=None,
                 radar_receiver=None, lidar_receiver=None):
        self._mti = mti_receiver
        self._gnss = gnss_receiver
        self._radar = radar_receiver
        self._lidar = lidar_receiver

    def sync(self, camera_frame) -> SyncedFrame:
        imu_frame = self._mti.get_nearest(camera_frame.timestamp_ns)

        if imu_frame is None:
            return None

        delta = abs(camera_frame.timestamp_ns - imu_frame.timestamp_ns)

        if delta > MAX_DELTA_NS:
            return None

        gnss = self._get_gnss()
        radar = self._get_radar(camera_frame.timestamp_ns)
        lidar = self._get_lidar(camera_frame.timestamp_ns)

        return SyncedFrame(
            camera=camera_frame,
            imu=imu_frame,
            camera_timestamp_ns=camera_frame.timestamp_ns,
            imu_timestamp_ns=imu_frame.timestamp_ns,
            delta_ns=delta,
            gnss=gnss,
            gnss_timestamp_ns=gnss.timestamp_ns,
            radar=radar,
            radar_timestamp_ns=radar.timestamp_ns,
            lidar=lidar,
            lidar_timestamp_ns=lidar.timestamp_ns,
        )

    def _get_gnss(self) -> GNSSData:
        if not self._gnss:
            return GNSSData()
        fix = self._gnss.latest()
        if not fix:
            return GNSSData()
        return GNSSData(
            latitude=fix["lat"],
            longitude=fix["lon"],
            altitude=fix["alt"],
            fix_quality=fix["fix_quality"],
            fix_type=fix["fix"],
            satellites=fix["satellites"],
            hdop=fix["hdop"],
            speed_mps=fix["speed"],
            course_deg=fix["course"],
            rtk_fixed=fix["rtk_fixed"],
            timestamp_ns=int(fix.get("timestamp_ns", 0)),
            utc_time=fix.get("utc_time", ""),
            gps_week=fix.get("gps_week", 0),
            gps_tow=fix.get("gps_tow", 0.0),
        )

    def _get_radar(self, camera_ts: int) -> RadarData:
        if not self._radar:
            return RadarData()
        frame = self._radar.get_nearest(camera_ts)
        if frame is None:
            return RadarData()
        delta = abs(frame.timestamp_ns - camera_ts)
        if delta > RADAR_MAX_DELTA_NS:
            return RadarData()
        return RadarData(
            points=frame.points,
            range_doppler=frame.range_doppler,
            range_profile=frame.range_profile,
            timestamp_ns=frame.timestamp_ns,
            frame_id=frame.frame_id,
            num_objects=frame.num_objects,
            num_range_bins=frame.num_range_bins,
            num_doppler_bins=frame.num_doppler_bins,
        )

    @property
    def ntrip_connected(self) -> bool:
        return bool(getattr(self._gnss, "ntrip_connected", False))

    @property
    def radar_streaming(self) -> bool:
        """Whether the radar data UART is delivering complete live frames."""
        return bool(getattr(self._radar, "streaming", False))

    @property
    def radar_rate(self) -> float:
        return float(getattr(self._radar, "rate", 0.0))

    def _get_lidar(self, camera_ts: int) -> LidarData:
        if not self._lidar:
            return LidarData()
        frame = self._lidar.get_nearest(camera_ts)
        if frame is None:
            return LidarData()
        delta = abs(frame.timestamp_ns - camera_ts)
        if delta > LIDAR_MAX_DELTA_NS:
            return LidarData()
        return LidarData(
            points_full=frame.points_full,
            reflectivity=frame.reflectivity,
            timestamp_ns=frame.timestamp_ns,
            frame_id=frame.frame_id,
            num_points=frame.num_points,
            complete=frame.complete,
        )

    def latest_lidar_frame(self):
        """Return the newest lidar frame for the live dashboard path.

        This intentionally does not apply the camera/IMU gate or the lidar
        matching tolerance.  ``LidarReceiver`` atomically replaces complete
        frame objects under its lock, so callers receive a stable frame while
        the receiver continues ingesting subsequent scans.
        """
        if not self._lidar:
            return None
        return self._lidar.get_frame()

    @property
    def lidar_streaming(self) -> bool:
        return bool(getattr(self._lidar, "streaming", False))

    @property
    def lidar_rate(self) -> float:
        return float(getattr(self._lidar, "rate", 0.0))
