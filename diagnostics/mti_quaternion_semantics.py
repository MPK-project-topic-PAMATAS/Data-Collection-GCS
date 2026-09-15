"""MTi-8 quaternion semantics diagnostic.

Reads directly from the MTi receiver and tests all quaternion interpretation
hypotheses against accelerometer gravity constraint.

Usage:
    .venv/bin/python diagnostics/mti_quaternion_semantics.py [duration_seconds]
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _euler_from_quat(q, convention):
    """Convert quaternion to (roll, pitch, yaw) degrees.

    convention: one of 'wxyz', 'xyzw'
    """
    if convention == "wxyz":
        w, x, y, z = q
    else:
        x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def _gravity_error_deg(quat_raw, accel_mean, convention, invert=False):
    """Compute angular error between quaternion-predicted and measured gravity.

    convention: 'wxyz' or 'xyzw' for the raw quaternion component order
    invert: if True, use R^T instead of R (tests world->body vs body->world)
    """
    if convention == "wxyz":
        w, x, y, z = quat_raw
        scipy_q = [x, y, z, w]  # scipy wants xyzw
    else:
        x, y, z, w = quat_raw
        scipy_q = [x, y, z, w]

    R = Rotation.from_quat(scipy_q).as_matrix()
    if invert:
        R = R.T

    # Quaternion predicts gravity in body frame: R^T @ [0,0,1]
    # (body->world rotation, gravity in world is [0,0,1])
    g_pred = R.T @ np.array([0.0, 0.0, 1.0])

    # Measured gravity in body frame
    g_meas = accel_mean / np.linalg.norm(accel_mean)

    cos_angle = np.clip(np.dot(g_pred, g_meas), -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def run(duration_s=10.0):
    from camera.zed_camera import ZEDCamera
    from mti8.receiver import MTiReceiver

    cam = ZEDCamera()
    cam.open()
    mti = MTiReceiver()
    mti.start()
    print("Sensors ready. Collecting data...\n")

    samples = []
    t0 = time.monotonic()

    try:
        while time.monotonic() - t0 < duration_s:
            frame = cam.grab()
            if frame is None:
                time.sleep(0.001)
                continue
            mti_frame = mti.get_frame()
            if mti_frame is None or mti_frame.quaternion is None:
                time.sleep(0.001)
                continue

            samples.append({
                "t": time.monotonic() - t0,
                "quat": list(mti_frame.quaternion),
                "accel": list(mti_frame.acceleration) if mti_frame.acceleration else [0, 0, 0],
                "gyro": list(mti_frame.gyroscope) if mti_frame.gyroscope else [0, 0, 0],
                "counter": mti_frame.packet_counter,
            })
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        mti.stop()
        cam.close()

    if not samples:
        print("ERROR: No samples captured.")
        return

    quats = np.array([s["quat"] for s in samples])
    accels = np.array([s["accel"] for s in samples])
    gyros = np.array([s["gyro"] for s in samples])

    print(f"Captured {len(samples)} samples over {samples[-1]['t']:.1f}s\n")

    # ── Raw quaternion stats ──────────────────────────────────────────
    print("=" * 60)
    print("RAW QUATERNION (from XDI 0x2010)")
    print("=" * 60)
    print(f"  Mean:  [{quats.mean(axis=0)[0]:+.6f}, {quats.mean(axis=0)[1]:+.6f}, "
          f"{quats.mean(axis=0)[2]:+.6f}, {quats.mean(axis=0)[3]:+.6f}]")
    norms = np.linalg.norm(quats, axis=1)
    print(f"  Norm:  mean={norms.mean():.8f}  std={norms.std():.8f}  "
          f"range=[{norms.min():.8f}, {norms.max():.8f}]")
    print(f"  XSens convention: q0(real), q1, q2, q3 = (w, x, y, z)")
    print(f"  Wire order:       [{quats.mean(axis=0)[0]:+.6f}, "
          f"{quats.mean(axis=0)[1]:+.6f}, {quats.mean(axis=0)[2]:+.6f}, "
          f"{quats.mean(axis=0)[3]:+.6f}]")
    print(f"                     q0(w)   q1(x)   q2(y)   q3(z)")

    # ── Accelerometer stats ───────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("ACCELEROMETER")
    print("=" * 60)
    a_mean = accels.mean(axis=0)
    a_std = accels.std(axis=0)
    a_norm = np.linalg.norm(a_mean)
    print(f"  Mean:  [{a_mean[0]:+.4f}, {a_mean[1]:+.4f}, {a_mean[2]:+.4f}] m/s^2")
    print(f"  Std:   [{a_std[0]:.4f}, {a_std[1]:.4f}, {a_std[2]:.4f}]")
    print(f"  Norm:  {a_norm:.4f} m/s^2 (expected ~9.81)")
    g_meas = a_mean / np.linalg.norm(a_mean)
    print(f"  Normalized gravity: [{g_meas[0]:+.6f}, {g_meas[1]:+.6f}, {g_meas[2]:+.6f}]")

    # ── Gyro stats ────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("GYROSCOPE (should be ~0 when stationary)")
    print("=" * 60)
    g_mean = gyros.mean(axis=0)
    g_std = gyros.std(axis=0)
    print(f"  Mean:  [{g_mean[0]:+.6f}, {g_mean[1]:+.6f}, {g_mean[2]:+.6f}] rad/s")
    print(f"  Std:   [{g_std[0]:.6f}, {g_std[1]:.6f}, {g_std[2]:.6f}]")

    # ── Quaternion hypothesis testing ─────────────────────────────────
    print(f"\n{'=' * 60}")
    print("QUATERNION INTERPRETATION HYPOTHESES")
    print("=" * 60)
    print(f"  XSens wire: q0={quats.mean(axis=0)[0]:+.6f}  "
          f"q1={quats.mean(axis=0)[1]:+.6f}  "
          f"q2={quats.mean(axis=0)[2]:+.6f}  "
          f"q3={quats.mean(axis=0)[3]:+.6f}")
    print()

    mean_q = quats.mean(axis=0)
    candidates = [
        ("wxyz, body->world (R)",     "wxyz", False),
        ("wxyz, world->body (R^T)",   "wxyz", True),
        ("xyzw, body->world (R)",     "xyzw", False),
        ("xyzw, world->body (R^T)",   "xyzw", True),
    ]

    print(f"  {'Candidate':<35s} {'Roll':>8s} {'Pitch':>8s} {'Yaw':>8s}  {'Grav Err':>10s}")
    print(f"  {'-'*35} {'-'*8} {'-'*8} {'-'*8}  {'-'*10}")

    best_err = 999.0
    best_name = ""
    for name, conv, invert in candidates:
        r, p, y = _euler_from_quat(mean_q, conv)
        grav_err = _gravity_error_deg(mean_q, a_mean, conv, invert)
        marker = ""
        if grav_err < best_err:
            best_err = grav_err
            best_name = name
        print(f"  {name:<35s} {r:+7.1f}\u00b0 {p:+7.1f}\u00b0 {y:+7.1f}\u00b0  {grav_err:8.2f}\u00b0")

    print(f"\n  BEST: {best_name}  (gravity error: {best_err:.2f}\u00b0)")

    # ── Summary ───────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("CONCLUSION")
    print("=" * 60)
    if best_err < 5.0:
        print(f"  The correct interpretation is: {best_name}")
        print(f"  Gravity prediction error: {best_err:.2f}\u00b0")
        print(f"  This means the XSens quaternion is (w,x,y,z) and represents")
        print(f"  body->world rotation (qLS as documented by XSens).")
    else:
        print(f"  WARNING: Best gravity error is {best_err:.2f}\u00b0 — investigate further.")
    print("=" * 60)


if __name__ == "__main__":
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    run(duration)
