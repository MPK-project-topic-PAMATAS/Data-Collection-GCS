import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "calibration"))
import extrinsic_solvers as xs  # noqa: E402
from fusion.transforms import (  # noqa: E402
    NOMINAL_ROTATION, Extrinsics, make_transform, matrix_to_rpy,
    pose_to_params, rpy_to_matrix, transform_points,
)

K = np.array([[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]])


def _ext(lidar_guess, radar_guess=(0, 0, 0, 0, 0, 0)):
    ext = Extrinsics(
        {"imu": np.eye(4),
         "camera": make_transform(NOMINAL_ROTATION["camera"], [0.5, 0.0, 0.3]),
         "lidar": xs.sensor_transform("lidar", lidar_guess),
         "radar": xs.sensor_transform("radar", radar_guess)},
        {"imu": True, "camera": True, "lidar": False, "radar": False},
        {"imu": {"translation_m": [0, 0, 0], "rpy_deg": [0, 0, 0]},
         "camera": {"translation_m": [0.5, 0, 0.3], "rpy_deg": [0, 0, 0]},
         "lidar": {"translation_m": list(lidar_guess[:3]), "rpy_deg": list(lidar_guess[3:])},
         "radar": {"translation_m": list(radar_guess[:3]), "rpy_deg": list(radar_guess[3:])}})
    return ext


def test_matrix_rpy_roundtrip():
    for rpy in [(10, -20, 30), (-45, 5, 170), (0, 0, 0)]:
        assert np.allclose(matrix_to_rpy(rpy_to_matrix(*rpy)), rpy, atol=1e-9)
    R = rpy_to_matrix(3, -7, 20) @ NOMINAL_ROTATION["radar"]
    t, rpy = pose_to_params("radar", make_transform(R, [1, 2, 3]))
    assert np.allclose(rpy, (3, -7, 20), atol=1e-9) and np.allclose(t, [1, 2, 3])


def test_lidar_camera_correspondences_recovers_pose():
    truth = np.array([0.20, 0.08, 0.35, 1.5, -2.0, 2.5])
    ext = _ext(truth + [0.07, -0.06, 0.05, 2.5, 2.0, -2.0])   # tape-measure-ish guess
    T_cl = np.linalg.inv(ext.T_vehicle_sensor("camera")) @ xs.sensor_transform("lidar", truth)
    rng = np.random.default_rng(0)
    # board hole centres at various ranges/angles, in the LIDAR frame
    pts_cam = np.c_[rng.uniform(-2, 2, 40), rng.uniform(-1, 1, 40), rng.uniform(3, 12, 40)]
    pts_l = transform_points(np.linalg.inv(T_cl), pts_cam)
    uv, _ = xs.project_to_pixels(K, pts_cam)
    uv += rng.normal(0, 0.5, uv.shape)
    res = xs.solve_lidar_camera_correspondences(ext, K, pts_l, uv, n_seeds=3, maxiter=80)
    assert np.allclose(res.translation_m, truth[:3], atol=0.02)
    assert np.allclose(res.rpy_deg, truth[3:], atol=0.3)
    assert res.rms < 1.0
    assert res.std_jacobian is not None and np.all(np.isfinite(res.std_jacobian))
    assert not res.at_bound


def test_too_few_correspondences_rejected():
    with pytest.raises(ValueError):
        xs.solve_lidar_camera_correspondences(_ext([0] * 6), K, np.zeros((3, 3)), np.zeros((3, 2)))


def test_nid_ranks_aligned_below_misaligned():
    rng = np.random.default_rng(1)
    a = rng.uniform(size=5000)
    assert xs.nid(a, 0.5 * a + 0.2) < 0.35        # monotone map: nearly determined
    assert xs.nid(a, rng.uniform(size=5000)) > 0.85


def _textured_scene(truth, ext, n=6000, seed=2):
    """Planar ground patch with lane-line-like texture, seen by lidar and camera."""
    rng = np.random.default_rng(seed)
    T_cl = np.linalg.inv(ext.T_vehicle_sensor("camera")) @ xs.sensor_transform("lidar", truth)
    # plane 8 m in front of the camera (camera frame), textured by (x, y)
    xy = np.c_[rng.uniform(-3, 3, n), rng.uniform(-1.8, 1.8, n)]
    pts_cam = np.c_[xy, np.full(n, 8.0)]
    tex = lambda x, y: (0.5 + 0.5 * np.sin(5 * x) * np.cos(7 * y) > 0.55).astype(float)
    inten = tex(xy[:, 0], xy[:, 1])
    H, W = 720, 1280
    vv, uu = np.mgrid[0:H, 0:W]
    x = (uu - K[0, 2]) * 8.0 / K[0, 0]
    y = (vv - K[1, 2]) * 8.0 / K[1, 1]
    img = tex(x, y)
    return transform_points(np.linalg.inv(T_cl), pts_cam), inten, img


def test_nid_targetless_improves_and_lands_near_truth():
    truth = np.array([0.20, 0.08, 0.35, 1.0, -1.0, 1.5])
    ext = _ext(truth + [0.0, 0.0, 0.0, 1.2, -1.0, 1.0])
    scans = [_textured_scene(truth, ext, seed=s) for s in (2, 3)]
    res = xs.solve_lidar_camera_nid(ext, K, scans, half_range=(0.0, 0.0, 0.0, 3, 3, 3),
                                    n_seeds=2, maxiter=40, popsize=10,
                                    fixed=("tx", "ty", "tz"))
    assert np.allclose(res.rpy_deg, truth[3:], atol=0.5)


def test_radar_lidar_reflectors():
    truth = np.array([0.10, -0.40, 0.05, 0.0, 0.0, 3.0])        # radar yawed 3 deg
    lid = np.array([0.0, 0.0, 0.0, 0, 0, 0])
    ext = _ext(lid, truth + [0.06, -0.05, 0.0, 0, 0, -2.0])
    rng = np.random.default_rng(3)
    pts_l = np.c_[rng.uniform(-6, 6, 14), rng.uniform(4, 25, 14), np.zeros(14)]
    # lidar frame is x fwd/y left: build it that way
    pts_l = np.c_[pts_l[:, 1], -pts_l[:, 0], pts_l[:, 2]]
    T_rl = np.linalg.inv(xs.sensor_transform("radar", truth)) @ ext.T_vehicle_sensor("lidar")
    pts_r = transform_points(T_rl, pts_l)
    pts_r = pts_r + rng.normal(0, [0.03, 0.03, 0.0], pts_r.shape)
    res = xs.solve_radar_lidar(ext, pts_r, pts_l, n_seeds=3, maxiter=80,
                               half_range=(0.12, 0.12, 0.0, 0, 0, 6))
    assert np.allclose(res.translation_m[:2], truth[:2], atol=0.05)
    assert res.rpy_deg[2] == pytest.approx(3.0, abs=0.5)


def test_radar_yaw_from_doppler():
    rng = np.random.default_rng(4)
    psi = np.radians(4.0)
    bearings, vrs, vref = [], [], []
    for _ in range(12):
        v = rng.uniform(3, 15)
        ang = rng.uniform(-1.0, 1.0, 40)                # radar-frame bearing from +y
        b = np.c_[np.sin(ang), np.cos(ang)]
        fx = b[:, 1] * np.cos(psi) + b[:, 0] * np.sin(psi)
        vrs.append(-v * fx + rng.normal(0, 0.05, 40))
        bearings.append(b)
        vref.append(v)
    assert xs.solve_radar_yaw_from_doppler(vrs, bearings, vref) == pytest.approx(4.0, abs=0.3)


def test_cli_writes_yaml_and_refuses_untrusted(tmp_path):
    import solve_extrinsics as cli
    from fusion.transforms import DEFAULT_PATH, load_extrinsics

    yaml_copy = tmp_path / "ext.yaml"
    original = Path(DEFAULT_PATH).read_text()
    yaml_copy.write_text(original)
    truth = np.array([-0.10, 0.05, 0.20, 0.0, 0.0, 2.0])
    ext = load_extrinsics(yaml_copy)
    ext.set_pose("lidar", [0.0, 0.0, 0.0], [0, 0, 0])
    rng = np.random.default_rng(5)
    pts_l = np.c_[rng.uniform(4, 25, 14), rng.uniform(-6, 6, 14), np.zeros(14)]
    T_rl = np.linalg.inv(xs.sensor_transform("radar", truth)) @ ext.T_vehicle_sensor("lidar")
    pts_r = transform_points(T_rl, pts_l)
    npz = tmp_path / "c.npz"
    np.savez(npz, pts_radar=pts_r, pts_lidar=pts_l)

    rc = cli.main(["radar-lidar", "--data", str(npz), "--extrinsics", str(yaml_copy),
                   "--range", "0.15", "0.15", "0", "0", "0", "6", "--seeds", "3", "--write"])
    assert rc == 0
    got = load_extrinsics(yaml_copy)
    assert got.measured["radar"]
    assert got.params["radar"]["rpy_deg"][2] == pytest.approx(2.0, abs=0.5)
    # too-small box => parameter pinned at the bound => untrusted, no write
    yaml_copy.write_text(original)
    before = original
    rc = cli.main(["radar-lidar", "--data", str(npz), "--extrinsics", str(yaml_copy),
                   "--range", "0.001", "0.001", "0", "0", "0", "0.1", "--seeds", "2", "--write"])
    assert rc == 1 and yaml_copy.read_text() == before
