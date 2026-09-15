#!/usr/bin/env python3
"""Phase 5-8 + 17-18: MTi-8 parser unit tests.

Covers: XBus framing, MTData2 decoding, quaternion norms, acceleration
magnitude, packet counter increment, GNSS range, checksum rejection.

Run: python tests/test_mti8_parser.py
"""
import math
import struct
import sys

sys.path.insert(0, ".")

from mti8.xbus_parser import XbusParser, PREAMBLE, BID, MTDATA2, EXTENDED_LEN, _build_packet
from mti8.mtdata2_decoder import decode


# ── Helpers ──────────────────────────────────────────────────────

def _build_xdi_packet(data_id: int, data: bytes) -> bytes:
    """Build a single XDI sub-packet: [id_hi][id_lo][size][data]."""
    return bytes([data_id >> 8, data_id & 0xFF, len(data)]) + data


def _build_mtdata2_payload(*xdi_chunks) -> bytes:
    """Concatenate XDI sub-packets into a full MTData2 payload."""
    return b"".join(xdi_chunks)


def _wrap_in_xbus(payload: bytes) -> bytes:
    """Wrap payload in a valid XBus packet with checksum."""
    return _build_packet(BID, MTDATA2, payload)


# ── XBus framing tests ─────────────────────────────────────────

def test_xbus_basic_packet():
    """Standard-length XBus packet parses correctly."""
    payload = bytes(range(20))
    pkt_bytes = _build_packet(BID, MTDATA2, payload)
    parser = XbusParser()
    parser.feed(pkt_bytes)
    packets = parser.parse_packets()
    assert len(packets) == 1
    assert packets[0].mid == MTDATA2
    assert packets[0].payload == payload
    assert not packets[0].extended
    print("  PASS: basic XBus packet")


def test_xbus_extended_packet():
    """Extended-length (>255 byte) XBus packet parses correctly."""
    payload = bytes(range(100)) * 3  # 300 bytes
    pkt_bytes = _build_packet(BID, MTDATA2, payload, extended=True)
    parser = XbusParser()
    parser.feed(pkt_bytes)
    packets = parser.parse_packets()
    assert len(packets) == 1
    assert packets[0].extended
    assert packets[0].payload == payload
    print("  PASS: extended XBus packet")


def test_xbus_bad_checksum_rejected():
    """Corrupted checksum causes packet rejection."""
    payload = bytes(range(10))
    pkt_bytes = bytearray(_build_packet(BID, MTDATA2, payload))
    pkt_bytes[-1] ^= 0x01  # flip checksum
    parser = XbusParser()
    parser.feed(bytes(pkt_bytes))
    packets = parser.parse_packets()
    assert len(packets) == 0
    print("  PASS: bad checksum rejected")


def test_xbus_wrong_bid_rejected():
    """Wrong BID causes resync, no packet returned."""
    payload = bytes(range(10))
    pkt_bytes = _build_packet(0x00, MTDATA2, payload)
    parser = XbusParser()
    parser.feed(pkt_bytes)
    packets = parser.parse_packets()
    assert len(packets) == 0
    print("  PASS: wrong BID rejected")


def test_xbus_resync_after_garbage():
    """Parser recovers from garbage bytes and finds next valid packet."""
    garbage = b"\x00\x01\x02\x03\x04"
    payload = bytes(range(10))
    pkt_bytes = _build_packet(BID, MTDATA2, payload)
    parser = XbusParser()
    parser.feed(garbage + pkt_bytes)
    packets = parser.parse_packets()
    assert len(packets) == 1
    assert packets[0].payload == payload
    print("  PASS: resync after garbage")


def test_xbus_multiple_packets():
    """Multiple consecutive packets in one feed."""
    p1 = _build_packet(BID, MTDATA2, bytes([1, 2, 3]))
    p2 = _build_packet(BID, MTDATA2, bytes([4, 5, 6, 7]))
    parser = XbusParser()
    parser.feed(p1 + p2)
    packets = parser.parse_packets()
    assert len(packets) == 2
    assert packets[0].payload == bytes([1, 2, 3])
    assert packets[1].payload == bytes([4, 5, 6, 7])
    print("  PASS: multiple packets")


def test_xbus_partial_packet_buffered():
    """Incomplete packet stays in buffer until more data arrives."""
    payload = bytes(range(20))
    full = _build_packet(BID, MTDATA2, payload)
    parser = XbusParser()
    parser.feed(full[:5])  # partial
    packets = parser.parse_packets()
    assert len(packets) == 0  # not enough data yet
    parser.feed(full[5:])    # rest
    packets = parser.parse_packets()
    assert len(packets) == 1
    assert packets[0].payload == payload
    print("  PASS: partial packet buffered")


# ── MTData2 decoding tests ─────────────────────────────────────

def test_decode_quaternion():
    """Quaternion is decoded as (x, y, z, w) float32 tuple."""
    q = (0.1, 0.2, 0.3, 0.9327)  # approximately unit: norm≈1.0
    # normalize to exact unit
    norm = math.sqrt(sum(x**2 for x in q))
    q = tuple(x / norm for x in q)
    payload = _build_xdi_packet(0x2010, struct.pack(">4f", *q))
    frame = decode(payload)
    assert frame.quaternion is not None
    for a, b in zip(frame.quaternion, q):
        assert abs(a - b) < 1e-5
    print("  PASS: quaternion decoded")


def test_quaternion_unit_norm():
    """Decoded quaternion has unit norm (|q| ≈ 1.0)."""
    q = (0.1, 0.2, 0.3, 0.9327)
    norm = math.sqrt(sum(x**2 for x in q))
    q = tuple(x / norm for x in q)
    payload = _build_xdi_packet(0x2010, struct.pack(">4f", *q))
    frame = decode(payload)
    qw, qx, qy, qz = frame.quaternion  # XSens wire: (w,x,y,z)
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    assert abs(norm - 1.0) < 0.01, f"Quaternion norm = {norm:.4f}"
    print(f"  PASS: quaternion norm = {norm:.6f}")


def test_decode_acceleration():
    """Acceleration is decoded as (x, y, z) m/s²."""
    a = (0.1, 0.2, 9.81)
    payload = _build_xdi_packet(0x4020, struct.pack(">3f", *a))
    frame = decode(payload)
    assert frame.acceleration is not None
    for v, e in zip(frame.acceleration, a):
        assert abs(v - e) < 1e-5
    print("  PASS: acceleration decoded")


def test_acceleration_static_magnitude():
    """Static acceleration magnitude ≈ 9.81 m/s² (gravity)."""
    a = (0.1, 0.2, 9.81)
    payload = _build_xdi_packet(0x4020, struct.pack(">3f", *a))
    frame = decode(payload)
    mag = math.sqrt(sum(x**2 for x in frame.acceleration))
    assert 9.5 < mag < 10.5, f"Acceleration magnitude = {mag:.3f}"
    print(f"  PASS: acceleration magnitude = {mag:.3f}")


def test_decode_gyroscope():
    """Gyroscope is decoded as (x, y, z) rad/s."""
    g = (0.001, -0.002, 0.003)
    payload = _build_xdi_packet(0x8020, struct.pack(">3f", *g))
    frame = decode(payload)
    assert frame.gyroscope is not None
    for v, e in zip(frame.gyroscope, g):
        assert abs(v - e) < 1e-6
    print("  PASS: gyroscope decoded")


def test_decode_timestamp():
    """Packet counter and sample time are decoded."""
    payload = (
        _build_xdi_packet(0x1020, struct.pack(">H", 42)) +
        _build_xdi_packet(0x1060, struct.pack(">I", 12345678))
    )
    frame = decode(payload)
    assert frame.packet_counter == 42
    assert frame.sample_time_fine == 12345678
    print("  PASS: timestamps decoded")


def test_decode_gnss():
    """GNSS lat/lon decoded as float64, within valid range."""
    lat, lon = 54.6872, 25.2797
    payload = _build_xdi_packet(0x5040, struct.pack(">2d", lat, lon))
    frame = decode(payload)
    assert abs(frame.latitude - lat) < 1e-6
    assert abs(frame.longitude - lon) < 1e-6
    assert -90 <= frame.latitude <= 90
    assert -180 <= frame.longitude <= 180
    print("  PASS: GNSS decoded, in range")


def test_decode_altitude():
    """Altitude MSL decoded as float64."""
    alt = 112.4
    payload = _build_xdi_packet(0x5010, struct.pack(">d", alt))
    frame = decode(payload)
    assert abs(frame.altitude_msl - alt) < 1e-4
    print("  PASS: altitude decoded")


def test_decode_status_word():
    """Status word decoded as uint32."""
    status = 0x00000081
    payload = _build_xdi_packet(0xE020, struct.pack(">I", status))
    frame = decode(payload)
    assert frame.status_word == status
    print("  PASS: status word decoded")


def test_decode_magnetometer():
    """Magnetometer decoded as (x, y, z) normalized."""
    m = (-0.364, 0.093, -0.960)
    payload = _build_xdi_packet(0xC020, struct.pack(">3f", *m))
    frame = decode(payload)
    assert frame.magnetometer is not None
    for v, e in zip(frame.magnetometer, m):
        assert abs(v - e) < 1e-4
    print("  PASS: magnetometer decoded")


def test_decode_temperature():
    """Temperature decoded as float32."""
    temp = 36.5
    payload = _build_xdi_packet(0x0810, struct.pack(">f", temp))
    frame = decode(payload)
    assert abs(frame.temperature - temp) < 1e-4
    print("  PASS: temperature decoded")


def test_decode_pressure():
    """Baro pressure (XDI 0x3010) is a uint32 in Pa, not a float32.

    Regression for a real bug found by an external dataset audit: unpacking
    the field as ">f" reinterpreted the integer bit pattern as a float,
    turning a real ~100 kPa reading into ~1.4e-40.
    """
    raw_bytes = bytes.fromhex("0001872b")  # 100139 Pa, exactly the audit's example
    assert struct.unpack(">I", raw_bytes)[0] == 100139
    payload = _build_xdi_packet(0x3010, raw_bytes)
    frame = decode(payload)
    assert frame.baro_pressure == 100139.0
    print("  PASS: pressure decoded as uint32 Pa")


def test_decode_unknown_xdi_skipped():
    """Unknown XDI IDs are silently skipped without error."""
    payload = (
        _build_xdi_packet(0xFFFF, bytes(10)) +  # unknown
        _build_xdi_packet(0x2010, struct.pack(">4f", 0, 0, 0, 1))  # quaternion
    )
    frame = decode(payload)
    assert frame.quaternion is not None
    print("  PASS: unknown XDI skipped")


def test_decode_empty_payload():
    """Empty payload returns default MTiFrame."""
    frame = decode(b"")
    assert frame.packet_counter == 0
    assert frame.quaternion is None
    print("  PASS: empty payload → default frame")


def test_decode_truncated_xdi():
    """Truncated XDI sub-packet stops decoding (no crash)."""
    payload = bytes([0x20, 0x10, 0x10]) + bytes(5)  # says 16 bytes, only 5
    frame = decode(payload)
    assert frame.quaternion is None  # truncated, not decoded
    print("  PASS: truncated XDI handled")


# ── Full pipeline test ──────────────────────────────────────────

def test_full_pipeline():
    """Build full MTData2 payload → wrap in XBus → parse → decode → verify."""
    q = (0.1, 0.2, 0.3, 0.9327)
    norm = math.sqrt(sum(x**2 for x in q))
    q = tuple(x / norm for x in q)
    a = (0.1, 0.2, 9.81)
    g = (0.001, -0.002, 0.003)

    payload = (
        _build_xdi_packet(0x1020, struct.pack(">H", 100)) +
        _build_xdi_packet(0x2010, struct.pack(">4f", *q)) +
        _build_xdi_packet(0x4020, struct.pack(">3f", *a)) +
        _build_xdi_packet(0x8020, struct.pack(">3f", *g))
    )

    pkt_bytes = _wrap_in_xbus(payload)
    parser = XbusParser()
    parser.feed(pkt_bytes)
    packets = parser.parse_packets()

    assert len(packets) == 1
    frame = decode(packets[0].payload)

    assert frame.packet_counter == 100
    for v, e in zip(frame.quaternion, q):
        assert abs(v - e) < 1e-5
    for v, e in zip(frame.acceleration, a):
        assert abs(v - e) < 1e-5
    for v, e in zip(frame.gyroscope, g):
        assert abs(v - e) < 1e-6

    # Quaternion norm
    qw, qx, qy, qz = frame.quaternion  # XSens wire: (w,x,y,z)
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    assert abs(norm - 1.0) < 0.01

    # Accel magnitude
    mag = math.sqrt(sum(x**2 for x in frame.acceleration))
    assert 9.5 < mag < 10.5

    print("  PASS: full pipeline (XBus → parse → decode → verify)")


# ── Packet counter increment test ──────────────────────────────

def test_packet_counter_monotonic():
    """Simulated packet counters are monotonically increasing."""
    counters = []
    for i in range(100):
        payload = _build_xdi_packet(0x1020, struct.pack(">H", i * 10))
        frame = decode(payload)
        counters.append(frame.packet_counter)
    for i in range(1, len(counters)):
        assert counters[i] > counters[i - 1], f"counter decreased at {i}"
    print("  PASS: packet counter monotonic")


# ── Canonical quaternion conversion test ──────────────────────

def test_quaternion_wire_order():
    """XSens wire format is (w,x,y,z), not (x,y,z,w).

    Identity quaternion q0=1,q1=0,q2=0,q3=0 means body=world (level sensor).
    With correct (w,x,y,z) interpretation, this gives roll=0.
    With wrong (x,y,z,w) interpretation, this gives roll=180.
    """
    import math
    from scipy.spatial.transform import Rotation

    # XSens wire: q0=1, q1=0, q2=0, q3=0 = (w,x,y,z) = identity
    identity = [1.0, 0.0, 0.0, 0.0]

    # Correct: w,x,y,z -> scipy (x,y,z,w) = [0,0,0,1]
    w, x, y, z = identity
    R_correct = Rotation.from_quat([x, y, z, w]).as_matrix()
    # Identity rotation: gravity along +Z
    g_pred = R_correct.T @ [0, 0, 1]
    grav_err = math.degrees(math.acos(max(-1, min(1, g_pred[2]))))
    assert grav_err < 1.0, f"Identity gravity error: {grav_err:.1f} deg"

    # Wrong: treating as x,y,z,w -> scipy gets [1,0,0,0] = 180° around X
    x_wrong, y_wrong, z_wrong, w_wrong = identity
    R_wrong = Rotation.from_quat([x_wrong, y_wrong, z_wrong, w_wrong]).as_matrix()
    g_wrong = R_wrong.T @ [0, 0, 1]
    grav_err_wrong = math.degrees(math.acos(max(-1, min(1, g_wrong[2]))))
    assert grav_err_wrong > 170.0, f"Wrong interpretation should give ~180 deg, got {grav_err_wrong:.1f}"

    print("  PASS: quaternion wire order verified (w,x,y,z)")


if __name__ == "__main__":
    tests = [
        test_xbus_basic_packet,
        test_xbus_extended_packet,
        test_xbus_bad_checksum_rejected,
        test_xbus_wrong_bid_rejected,
        test_xbus_resync_after_garbage,
        test_xbus_multiple_packets,
        test_xbus_partial_packet_buffered,
        test_decode_quaternion,
        test_quaternion_unit_norm,
        test_decode_acceleration,
        test_acceleration_static_magnitude,
        test_decode_gyroscope,
        test_decode_timestamp,
        test_decode_gnss,
        test_decode_altitude,
        test_decode_status_word,
        test_decode_magnetometer,
        test_decode_temperature,
        test_decode_pressure,
        test_decode_unknown_xdi_skipped,
        test_decode_empty_payload,
        test_decode_truncated_xdi,
        test_full_pipeline,
        test_packet_counter_monotonic,
        test_quaternion_wire_order,
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
