import numpy as np
import pytest

from config.settings import RADAR_TO_CAMERA_OFFSET_M
from fusion.transforms import (
    NOMINAL_ROTATION, Extrinsics, load_extrinsics, invert, make_transform,
    rpy_to_matrix, transform_points,
)


def test_nominal_rotations_are_proper():
    for R in NOMINAL_ROTATION.values():
        assert np.allclose(R @ R.T, np.eye(3))
        assert np.isclose(np.linalg.det(R), 1.0)


def test_axis_mappings():
    # camera forward (z) is vehicle forward (x); camera right (x) is vehicle -y
    Rc = NOMINAL_ROTATION["camera"]
    assert np.allclose(Rc @ [0, 0, 1], [1, 0, 0])
    assert np.allclose(Rc @ [1, 0, 0], [0, -1, 0])
    assert np.allclose(Rc @ [0, 1, 0], [0, 0, -1])
    # radar forward (y) is vehicle forward (x); radar right (x) is vehicle -y
    Rr = NOMINAL_ROTATION["radar"]
    assert np.allclose(Rr @ [0, 1, 0], [1, 0, 0])
    assert np.allclose(Rr @ [1, 0, 0], [0, -1, 0])


def test_invert_roundtrip():
    T = make_transform(rpy_to_matrix(10, -20, 30), [1, 2, 3])
    assert np.allclose(T @ invert(T), np.eye(4))
    p = np.array([[1.0, 2, 3], [-4, 5, 6]])
    assert np.allclose(transform_points(invert(T), transform_points(T, p)), p)


def test_yaw_90_turns_forward_to_left():
    assert np.allclose(rpy_to_matrix(yaw_deg=90) @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_chain_radar_to_camera_point():
    ext = Extrinsics(
        {"imu": np.eye(4),
         "camera": make_transform(NOMINAL_ROTATION["camera"], [1.0, 0.0, 0.5]),
         "radar": make_transform(NOMINAL_ROTATION["radar"], [1.0, -0.1, 0.5])},
        {"imu": True, "camera": True, "radar": True})
    # radar point 5 m ahead, 1 m right: radar (x=1, y=5, z=0)
    cam = ext.transform([1.0, 5.0, 0.0], "radar", "camera")
    # camera sits 0.1 m to the radar's left: point is 1.1 m right, 5 m forward
    assert np.allclose(cam, [1.1, 0.0, 5.0])


def test_shipped_extrinsics_match_legacy_radar_camera_offset():
    ext = load_extrinsics()
    assert np.allclose(ext.radar_to_camera_offset(), RADAR_TO_CAMERA_OFFSET_M, atol=1e-9)


def test_strict_rejects_placeholders():
    with pytest.raises(ValueError):
        load_extrinsics(strict=True)


def test_se3_projection_matches_legacy_when_parallel():
    from radar.frame import RadarPoint
    from radar.projection import project_point, project_points_se3
    ext = load_extrinsics()
    p = RadarPoint(x=0.7, y=3.0, z=0.2)
    legacy = project_point(p, 700, 700, 640, 360, RADAR_TO_CAMERA_OFFSET_M)
    uv = project_points_se3([[p.x, p.y, p.z]], ext, 700, 700, 640, 360)[0]
    assert np.allclose(uv, legacy, atol=0.51)
