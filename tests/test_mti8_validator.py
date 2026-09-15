#!/usr/bin/env python3
"""MTi-8 sample validator tests — covers all 8 required scenarios.

Test 1: valid sample → valid sample — state updates
Test 2: valid sample → no packet — previous state retained
Test 3: valid sample → zero/default packet — rejected, previous retained
Test 4: valid quaternion → invalid quaternion — rejected
Test 5: yaw +179° → yaw -179° — no false 358° jump
Test 6: small physical quaternion change — accepted
Test 7: missing packet sequence number — dropped counter increments
Test 8: invalid sample → valid sample — immediately restores normal state

Run: python tests/test_mti8_validator.py
"""
import math
import sys
import time

sys.path.insert(0, ".")

from mti8.frame import MTiFrame
from mti8.validator import MTiValidator, MTiSampleState, _quat_norm, _quat_multiply, _quat_conjugate


# ── Helpers ──────────────────────────────────────────────────────

def _make_frame(
    quat=(1.0, 0.0, 0.0, 0.0),
    accel=(0.1, 0.2, 9.81),
    gyro=(0.001, -0.002, 0.003),
    packet_counter=1,
    timestamp_ns=None,
):
    """Build a realistic MTiFrame."""
    if timestamp_ns is None:
        timestamp_ns = time.monotonic_ns()
    return MTiFrame(
        packet_counter=packet_counter,
        sample_time_fine=packet_counter * 50000,
        quaternion=quat,
        acceleration=accel,
        gyroscope=gyro,
        timestamp_ns=timestamp_ns,
    )


def _euler_from_quat(q):
    """XSens (w,x,y,z) to (roll, pitch, yaw) in degrees."""
    w, x, y, z = q
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))
    return roll, pitch, yaw


def _euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Euler (degrees) to XSens (w,x,y,z) quaternion."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


# ── Test 1: valid → valid — state updates ───────────────────────

def test_valid_to_valid():
    v = MTiValidator()
    f1 = _make_frame(quat=(1.0, 0.0, 0.0, 0.0), packet_counter=1)
    f2 = _make_frame(quat=(0.9659, 0.0, 0.2588, 0.0), packet_counter=2)

    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.VALID
    assert frame2.packet_counter == 2
    assert v.counters.valid_packets == 2
    print("  PASS: test_valid_to_valid")


# ── Test 2: valid → no packet — previous retained ───────────────

def test_valid_to_no_packet():
    v = MTiValidator()
    f1 = _make_frame(quat=(1.0, 0.0, 0.0, 0.0), packet_counter=10)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    frame2, state2 = v.validate(None)
    assert state2 == MTiSampleState.NO_PACKET
    # Must return the last valid frame
    assert frame2.packet_counter == 10
    assert v.counters.invalid_packets == 1
    # Counters: 1 total → 1 valid, then 1 total → 1 invalid
    assert v.counters.total_packets == 2
    print("  PASS: test_valid_to_no_packet")


# ── Test 3: valid → zero/default packet — rejected ──────────────

def test_valid_to_zero_packet():
    v = MTiValidator()
    f1 = _make_frame(quat=(1.0, 0.0, 0.0, 0.0), packet_counter=20)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    # Build a default MTiFrame (all zeros, no timestamp, no counter)
    zero_frame = MTiFrame()
    frame2, state2 = v.validate(zero_frame)
    assert state2 == MTiSampleState.INVALID_PACKET
    # Must return the last valid frame, not the zero frame
    assert frame2.packet_counter == 20
    assert frame2.quaternion == (1.0, 0.0, 0.0, 0.0)
    print("  PASS: test_valid_to_zero_packet")


# ── Test 4: valid quaternion → invalid quaternion — rejected ─────

def test_valid_to_invalid_quaternion():
    v = MTiValidator()
    f1 = _make_frame(quat=(1.0, 0.0, 0.0, 0.0), packet_counter=30)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    # Quaternion with NaN
    f2 = _make_frame(
        quat=(float('nan'), 0.0, 0.0, 0.0),
        packet_counter=31,
    )
    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.INVALID_ORIENTATION
    assert frame2.packet_counter == 30  # last valid retained

    # Quaternion with bad norm (>tolerance)
    f3 = _make_frame(
        quat=(2.0, 0.0, 0.0, 0.0),  # norm = 2.0
        packet_counter=32,
    )
    frame3, state3 = v.validate(f3)
    assert state3 == MTiSampleState.INVALID_ORIENTATION
    assert frame3.packet_counter == 30  # last valid retained
    print("  PASS: test_valid_to_invalid_quaternion")


# ── Test 5: yaw +179° → yaw -179° — no false 358° jump ─────────

def test_yaw_wrap_179_to_minus179():
    """Two quaternions 2° apart but wrapping ±180° must NOT trigger
    suspicious rotation."""
    v = MTiValidator(quat_jump_tol=0.5, omega_max_rad_s=10.0)

    q179p = _euler_to_quat(0.0, 0.0, 179.0)
    q179n = _euler_to_quat(0.0, 0.0, -179.0)

    f1 = _make_frame(quat=q179p, packet_counter=40, timestamp_ns=1_000_000_000)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    # 2° rotation, should be accepted
    f2 = _make_frame(quat=q179n, packet_counter=41, timestamp_ns=1_000_005_000)  # 5ms later
    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.VALID
    assert v.counters.suspicious_packets == 0

    # Verify the angular difference is small
    dq = _quat_multiply(_quat_conjugate(q179p), q179n)
    theta = 2.0 * math.acos(max(-1.0, min(1.0, abs(dq[0]))))
    assert math.degrees(theta) < 5.0, f"Expected ~2°, got {math.degrees(theta):.1f}°"
    print("  PASS: test_yaw_wrap_179_to_minus179")


# ── Test 6: small physical quaternion change — accepted ──────────

def test_small_quaternion_change():
    """Small rotation (5°) must be accepted."""
    v = MTiValidator(quat_jump_tol=0.5, omega_max_rad_s=10.0)

    q0 = _euler_to_quat(0.0, 0.0, 0.0)
    q5 = _euler_to_quat(0.0, 0.0, 5.0)

    f1 = _make_frame(quat=q0, packet_counter=50, timestamp_ns=1_000_000_000)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    f2 = _make_frame(quat=q5, packet_counter=51, timestamp_ns=1_000_005_000)
    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.VALID
    assert v.counters.suspicious_packets == 0
    print("  PASS: test_small_quaternion_change")


# ── Test 7: missing packet sequence number — counter increments ──

def test_dropped_packet_counter():
    v = MTiValidator()

    f1 = _make_frame(packet_counter=100)
    v.validate(f1)

    # Skip 101, 102, 103 — jump to 104
    f2 = _make_frame(packet_counter=104)
    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.VALID
    assert v.counters.dropped_packets == 3  # 101, 102, 103
    print("  PASS: test_dropped_packet_counter")


# ── Test 8: invalid → valid — immediately restores normal state ──

def test_invalid_to_valid_restores():
    v = MTiValidator()

    f1 = _make_frame(quat=(1.0, 0.0, 0.0, 0.0), packet_counter=60)
    frame1, state1 = v.validate(f1)
    assert state1 == MTiSampleState.VALID

    # Reject an invalid frame
    f_bad = _make_frame(quat=(float('nan'), 0.0, 0.0, 0.0), packet_counter=61)
    frame_bad, state_bad = v.validate(f_bad)
    assert state_bad == MTiSampleState.INVALID_ORIENTATION
    assert frame_bad.packet_counter == 60  # last valid

    # Next valid frame must be accepted immediately
    f2 = _make_frame(quat=(0.9659, 0.0, 0.2588, 0.0), packet_counter=62)
    frame2, state2 = v.validate(f2)
    assert state2 == MTiSampleState.VALID
    assert frame2.packet_counter == 62  # new valid, not the old one
    assert v.counters.valid_packets == 2
    print("  PASS: test_invalid_to_valid_restores")


# ── Run all ─────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [
        test_valid_to_valid,
        test_valid_to_no_packet,
        test_valid_to_zero_packet,
        test_valid_to_invalid_quaternion,
        test_yaw_wrap_179_to_minus179,
        test_small_quaternion_change,
        test_dropped_packet_counter,
        test_invalid_to_valid_restores,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            print(f"  FAIL: {t.__name__}: {e}")
            failed += 1
    print(f"\n{'=' * 40}")
    print(f"Results: {passed} passed, {failed} failed")
    if failed:
        sys.exit(1)
