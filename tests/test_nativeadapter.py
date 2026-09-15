"""The native facades must mirror the per-sensor Python adapter interfaces
(e.g. Synchronizer calls ``get_nearest(ts)``, run_gcs calls ``grab()`` /
``latest()``), returning the same dataclass types so downstream code is
unchanged.

These tests run against synthetic native frames registered as ``NativeAcq``
"native frames" through the shim; no hardware or manager is started.
"""

import sys
import types

# A minimal pyzed stub so importing nativeadapter -> radar.frame ->
# radar.receiver -> config.settings does not fail in the test environment.
def _install_pyzed_stub():
    sl = types.SimpleNamespace()
    sl.RESOLUTION = types.SimpleNamespace(HD720=2, HD2K=5)
    sl.DEPTH_MODE = types.SimpleNamespace(NEURAL_LIGHT=1, PERFORMANCE=2)
    sl.UNIT = types.SimpleNamespace(METER=1)
    sl.COORDINATE_SYSTEM = types.SimpleNamespace(RIGHT_HANDED_Z_UP=1)
    pz = types.ModuleType("pyzed")
    pz.sl = sl
    sys.modules.setdefault("pyzed", pz)
    sys.modules.setdefault("pyzed.sl", sl)


_install_pyzed_stub()

import nativeadapter as na  # noqa: E402


def _native_mti_frame(**kw):
    base = dict(
        packet_counter=1, sample_time_fine=2, sample_time_coarse=3,
        quaternion=[0, 0, 0, 1], euler=[1.0, 2.0, 3.0],
        acceleration=[0.0, 0.0, 9.8], gyroscope=[0.0, 0.0, 0.1],
        magnetometer=[1.0, 2.0, 3.0], velocity=[0.1, 0.2, 0.3],
        latitude=1.0, longitude=2.0, altitude_msl=3.0, altitude_ellipsoid=4.0,
        temperature=25.0, baro_pressure=1000.0, status_word=7, status_byte=3,
        timestamp_ns=1234,
    )
    base.update(kw)
    return types.SimpleNamespace(**base)


def _native_radar_frame(**kw):
    pt = types.SimpleNamespace(x=1.0, y=2.0, z=3.0, velocity_mps=0.5)
    base = dict(
        timestamp_ns=99, frame_id=7, num_objects=1,
        points=[pt], range_doppler=[1, 2, 3, 4], range_profile=[5, 6],
        num_range_bins=2, num_doppler_bins=2,
    )
    base.update(kw)
    return types.SimpleNamespace(**base)


def test_mti_conversion_preserves_data_and_type():
    frame = na._to_mti_frame(_native_mti_frame())
    assert type(frame).__name__ == "MTiFrame"
    assert tuple(frame.quaternion) == (0, 0, 0, 1)      # (w, x, y, z)
    assert frame.acceleration == (0.0, 0.0, 9.8)
    assert frame.timestamp_ns == 1234
    assert frame.latitude == 1.0


def test_radar_conversion_preserves_points_and_heats():
    frame = na._to_radar_frame(_native_radar_frame())
    assert type(frame).__name__ == "RadarFrame"
    assert frame.points[0].x == 1.0 and frame.points[0].velocity_mps == 0.5
    assert frame.range_doppler.shape == (2, 2)
    assert frame.range_profile.shape == (2,)
    assert frame.frame_id == 7


def test_lidar_conversion_points_and_reflectivity():
    nf = types.SimpleNamespace(
        frame_id=3, timestamp_ns=50, num_points=2, complete=True,
        points=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], reflectivity=[0.5, 0.9],
    )
    frame = na._to_lidar_frame(nf)
    assert type(frame).__name__ == "LidarFrame"
    assert frame.points_full.shape == (2, 3)
    assert frame.num_points == 2 and frame.complete is True
    assert frame.reflectivity.tolist() == [0.5, 0.9]


def test_gnss_conversion_dict_shape_matches_python_receiver():
    gf = types.SimpleNamespace(
        timestamp_ns=10, latitude=1.1, longitude=2.2, altitude=3.3,
        fix_quality=4, satellites=20, hdop=0.8, speed_mps=5.0,
        course_deg=90.0, rtk_fixed=True, has_fix=True,
    )
    fix = na._to_gnss_dict(gf)
    assert fix["lat"] == 1.1 and fix["lon"] == 2.2
    assert fix["rtk_fixed"] is True
    assert fix["timestamp_ns"] == 10
    # keys the Synchronizer._get_gnss reads
    for k in ("alt", "fix", "fix_quality", "satellites", "hdop", "speed",
              "course", "utc_time", "gps_week", "gps_tow"):
        assert k in fix


def test_facades_construct_from_manager_without_starting():
    cfg = na._sb.AcquisitionConfig()
    mgr = na._sb.AcquisitionManager(cfg)
    acq = na.NativeAcquisition(enable_zed=False, enable_mti8=False,
                               enable_radar=False, enable_ouster=False,
                               enable_gnss=False)
    # Facades bind to the manager even with nothing enabled.
    assert acq.camera.is_opened() is False
    assert acq.mti.get_frame() is None
    assert acq.gnss.latest() is None
    assert acq.radar.get_frame() is None
    assert acq.lidar.get_frame() is None


def test_settings_are_reflected_into_native_configs():
    # config.settings (stubbed pyzed) sets CAMERA_RESOLUTION=HD720(2),
    # MTI_BAUDRATE=2000000, RADAR_DATA_BAUD=921600, LIDAR_HOSTNAME.
    acq = na.NativeAcquisition(enable_zed=False, enable_mti8=False,
                               enable_radar=False, enable_ouster=False,
                               enable_gnss=False)
    assert acq.z.resolution == 2
    assert acq.m.baudrate == 2000000
    assert acq.r.data_baud == 921600
    assert acq.o.hostname == "169.254.197.185"
