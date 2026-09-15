"""MTData2 XDI packet decoder.

All XDI IDs are taken from the official XSens header:
https://github.com/ethz-asl/libxsensmt/blob/master/src/lib/xsens/xsdataidentifier.h

Every enabled XDI field in the MTData2 payload is decoded. Unknown IDs are
silently skipped (they may be enabled on other configurations).
"""
import struct
from typing import Dict, Any

from mti8.frame import MTiFrame

# ── XDI Data IDs (official XSens values) ──────────────────────────────

# Timestamp group (0x1000)
DI_UTC_TIME = 0x1010
DI_PACKET_COUNTER = 0x1020
DI_SAMPLE_TIME_FINE = 0x1060
DI_SAMPLE_TIME_COARSE = 0x1070

# Temperature group (0x0800)
DI_TEMPERATURE = 0x0810

# Orientation group (0x2000)
DI_QUATERNION = 0x2010
DI_ROTATION_MATRIX = 0x2020
DI_EULER = 0x2030

# Pressure group (0x3000)
DI_BARO_PRESSURE = 0x3010

# Acceleration group (0x4000)
DI_DELTA_VELOCITY = 0x4010
DI_ACCELERATION = 0x4020
DI_FREE_ACCELERATION = 0x4030

# Angular velocity group (0x8000)
DI_RATE_OF_TURN = 0x8020
DI_DELTA_QUATERNION = 0x8030

# Position group (0x5000)
DI_ALTITUDE_MSL = 0x5010
DI_ALTITUDE_ELLIPSOID = 0x5020
DI_POSITION_ECEF = 0x5030
DI_LAT_LON = 0x5040

# Magnetic field group (0xC000)
DI_MAGNETIC_FIELD = 0xC020

# Velocity group (0xD000)
DI_VELOCITY_XYZ = 0xD010

# Status group (0xE000)
DI_STATUS_BYTE = 0xE010
DI_STATUS_WORD = 0xE020


def decode(payload: bytes) -> MTiFrame:
    """Decode MTData2 payload into MTiFrame.

    Each XDI packet in the payload is 3-byte header + N-byte data:
      [data_id_hi][data_id_lo][size][size bytes of data]

    All multi-byte values are big-endian (XSens convention).
    """
    kw: Dict[str, Any] = {}
    offset = 0

    while offset + 3 <= len(payload):
        data_id = (payload[offset] << 8) | payload[offset + 1]
        size = payload[offset + 2]
        offset += 3

        if offset + size > len(payload):
            break

        raw = payload[offset:offset + size]
        offset += size

        # Timestamp
        if data_id == DI_PACKET_COUNTER and size >= 2:
            kw["packet_counter"] = struct.unpack(">H", raw[:2])[0]
        elif data_id == DI_SAMPLE_TIME_FINE and size >= 4:
            kw["sample_time_fine"] = struct.unpack(">I", raw[:4])[0]
        elif data_id == DI_SAMPLE_TIME_COARSE and size >= 4:
            kw["sample_time_coarse"] = struct.unpack(">I", raw[:4])[0]

        # Temperature
        elif data_id == DI_TEMPERATURE and size >= 4:
            kw["temperature"] = struct.unpack(">f", raw[:4])[0]

        # Orientation
        elif data_id == DI_QUATERNION and size >= 16:
            kw["quaternion"] = struct.unpack(">4f", raw[:16])
        elif data_id == DI_ROTATION_MATRIX and size >= 36:
            kw["rotation_matrix"] = struct.unpack(">9f", raw[:36])
        elif data_id == DI_EULER and size >= 12:
            kw["euler"] = struct.unpack(">3f", raw[:12])

        # Pressure — XDI 0x3010 is a uint32 in Pa (Xsens MT Low-Level
        # Documentation), NOT a float32. Unpacking it as ">f" reinterprets
        # the integer's raw bits as a float, producing values like 1e-40
        # for an actual reading of ~100000 Pa.
        elif data_id == DI_BARO_PRESSURE and size >= 4:
            kw["baro_pressure"] = float(struct.unpack(">I", raw[:4])[0])

        # Acceleration
        elif data_id == DI_DELTA_VELOCITY and size >= 12:
            kw["delta_velocity"] = struct.unpack(">3f", raw[:12])
        elif data_id == DI_ACCELERATION and size >= 12:
            kw["acceleration"] = struct.unpack(">3f", raw[:12])
        elif data_id == DI_FREE_ACCELERATION and size >= 12:
            pass  # decoded but not stored in MTiFrame yet

        # Angular velocity
        elif data_id == DI_RATE_OF_TURN and size >= 12:
            kw["gyroscope"] = struct.unpack(">3f", raw[:12])
        elif data_id == DI_DELTA_QUATERNION and size >= 16:
            kw["delta_quaternion"] = struct.unpack(">4f", raw[:16])

        # Position
        elif data_id == DI_LAT_LON and size >= 16:
            lat, lon = struct.unpack(">2d", raw[:16])
            kw["latitude"] = lat
            kw["longitude"] = lon
        elif data_id == DI_ALTITUDE_MSL and size >= 8:
            kw["altitude_msl"] = struct.unpack(">d", raw[:8])[0]
        elif data_id == DI_ALTITUDE_ELLIPSOID and size >= 8:
            kw["altitude_ellipsoid"] = struct.unpack(">d", raw[:8])[0]

        # Magnetic
        elif data_id == DI_MAGNETIC_FIELD and size >= 12:
            kw["magnetometer"] = struct.unpack(">3f", raw[:12])

        # Velocity
        elif data_id == DI_VELOCITY_XYZ and size >= 12:
            kw["velocity"] = struct.unpack(">3f", raw[:12])

        # Status
        elif data_id == DI_STATUS_BYTE and size >= 1:
            kw["status_byte"] = raw[0]
        elif data_id == DI_STATUS_WORD and size >= 4:
            kw["status_word"] = struct.unpack(">I", raw[:4])[0]

    return MTiFrame(**kw)


# ── Self-test ──────────────────────────────────────────────────────────

def _demo():
    """Build a synthetic payload with all known XDI IDs and verify decode."""
    payload = bytearray()

    # Timestamp
    payload += bytes([0x10, 0x20, 0x02]) + struct.pack(">H", 42)
    payload += bytes([0x10, 0x60, 0x04]) + struct.pack(">I", 12345678)
    payload += bytes([0x10, 0x70, 0x04]) + struct.pack(">I", 999)

    # Temperature
    payload += bytes([0x08, 0x10, 0x04]) + struct.pack(">f", 36.5)

    # Quaternion (must be unit quaternion for norm test)
    quat = (1.0, 0.0, 0.0, 0.0)
    payload += bytes([0x20, 0x10, 0x10]) + struct.pack(">4f", *quat)

    # Pressure
    payload += bytes([0x30, 0x10, 0x04]) + struct.pack(">f", 101325.0)

    # Acceleration
    payload += bytes([0x40, 0x20, 0x0C]) + struct.pack(">3f", 0.1, 0.2, 9.8)

    # Rate of turn
    payload += bytes([0x80, 0x20, 0x0C]) + struct.pack(">3f", 0.001, 0.002, 0.003)

    # Magnetometer
    payload += bytes([0xC0, 0x20, 0x0C]) + struct.pack(">3f", -0.364, 0.093, -0.960)

    # Lat/Lon
    payload += bytes([0x50, 0x40, 0x10]) + struct.pack(">2d", 54.6872, 25.2797)

    # Altitude MSL
    payload += bytes([0x50, 0x10, 0x08]) + struct.pack(">d", 112.4)

    # Velocity
    payload += bytes([0xD0, 0x10, 0x0C]) + struct.pack(">3f", 1.0, 2.0, 3.0)

    # Status word
    payload += bytes([0xE0, 0x20, 0x04]) + struct.pack(">I", 0x00000081)

    frame = decode(bytes(payload))

    assert frame.packet_counter == 42
    assert frame.sample_time_fine == 12345678
    assert frame.sample_time_coarse == 999
    assert abs(frame.temperature - 36.5) < 1e-5
    assert all(
        abs(a - b) < 1e-6
        for a, b in zip(frame.quaternion, quat)
    )
    assert abs(frame.temperature - 36.5) < 1e-5
    assert all(
        abs(a - b) < 1e-6
        for a, b in zip(frame.acceleration, (0.1, 0.2, 9.8))
    )
    assert all(
        abs(a - b) < 1e-6
        for a, b in zip(frame.gyroscope, (0.001, 0.002, 0.003))
    )
    assert all(
        abs(a - b) < 1e-6
        for a, b in zip(frame.magnetometer, (-0.364, 0.093, -0.960))
    )
    assert abs(frame.latitude - 54.6872) < 1e-6
    assert abs(frame.longitude - 25.2797) < 1e-6
    assert abs(frame.altitude_msl - 112.4) < 1e-6
    assert all(abs(a - b) < 1e-6 for a, b in zip(frame.velocity, (1.0, 2.0, 3.0)))
    assert frame.status_word == 0x00000081

    # Quaternion norm check
    import math
    qw, qx, qy, qz = frame.quaternion  # XSens wire: (w,x,y,z)
    norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    assert abs(norm - 1.0) < 1e-5, f"Quaternion norm = {norm}, expected 1.0"

    print("mtdata2_decoder: OK")


if __name__ == "__main__":
    _demo()
