"""TLV parser for TI AWR1843BOOST mmWave data frames (MMWAVE SDK 3).

Frame layout (per TI MMW demo `MmwDemo_transmitProcessedOutput`):
  [Frame header: 40B] [TLV0] [TLV1] ...

Frame header (40 B):
  magic (8B, bytes 0x02 0x01 0x04 0x03 0x06 0x05 0x08 0x07)
  version (u32) | totalPacketLen (u32) | platform (u32) | frameNumber (u32) |
  timeCpuCycles (u32) | numDetectedObj (u32) | numTLVs (u32) |
  subFrameNumber (u32)
  => 8 + 8*4 = 40 bytes.  totalPacketLen (in bytes, incl. header) at offset 12.

TLV:
  type (u32) | length (u32) | payload[length]

Detected Objects TLV (type=1):
  numDetectedObj records of 4 x float32: x, y, z, velocity (meters, m/s)
  TLV length == numDetectedObj * 16 (data only).

Range Profile TLV (type=2):
  uint16[] magnitude (one value per configured range bin)

Range-Doppler TLV (type=5):
  uint16[] matrix (numRangeBins * numDopplerBins, range-major)

In the AWR1843 mmWave SDK 3 output protocol, a TLV's ``length`` is the byte
length of its payload *only*; the 8-byte TLV header is separate.  The bin
counts are configured by the radar profile rather than included in these TLV
payloads.
"""

from __future__ import annotations

import struct
from typing import List, Optional

import numpy as np

from radar.frame import RadarFrame, RadarPoint

_MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"

_HEADER_FMT = "<8sIIIIIIII"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)

_TLV_DETECTED_POINTS = 1
_TLV_RANGE_PROFILE = 2
_TLV_RANGE_DOPPLER = 5

_POINT_SIZE_BYTES = 16  # x, y, z, velocity as float32
_POINT_FMT = "<ffff"


def parse_frame_header(data: bytes) -> Optional[dict]:
    """Parse the 40-byte AWR1843 frame header. Returns dict or None if bad."""
    if len(data) < _HEADER_SIZE:
        return None
    magic, version, total_packet_len, platform, frame_number, \
        time_cpu_cycles, num_detected_obj, num_tlvs, sub_frame_number = \
        struct.unpack_from(_HEADER_FMT, data, 0)
    if magic != _MAGIC:
        return None
    return {
        "total_packet_len": total_packet_len,
        "version": version,
        "platform": platform,
        "frame_number": frame_number,
        "time_cpu_cycles": time_cpu_cycles,
        "num_detected_obj": num_detected_obj,
        "num_tlvs": num_tlvs,
        "sub_frame_number": sub_frame_number,
    }


def parse_tlvs(payload: bytes, num_detected_obj: int,
               num_range_bins: int = 256,
               num_doppler_bins: int = 16) -> dict:
    """Parse TLVs from the payload (bytes after the frame header).

    Returns dict with optional 'points' (List[RadarPoint]),
    'range_doppler' (np.ndarray), 'num_range_bins', 'num_doppler_bins'.
    """
    result: dict = {}
    offset = 0
    n = len(payload)

    while offset + 8 <= n:
        # AWR1843 SDK 3 TLV length excludes this 8-byte header.
        tlv_type, tlv_len = struct.unpack_from("<II", payload, offset)
        body = offset + 8
        if body + tlv_len > n:
            break

        if tlv_type == _TLV_DETECTED_POINTS and "points" not in result:
            result["points"] = _parse_points(
                payload, body, num_detected_obj
            )

        elif tlv_type == _TLV_RANGE_DOPPLER and "range_doppler" not in result:
            rd = _parse_range_doppler(
                payload, body, tlv_len, num_range_bins, num_doppler_bins
            )
            if rd is not None:
                result["range_doppler"] = rd
                result["num_range_bins"] = rd.shape[0]
                result["num_doppler_bins"] = rd.shape[1]

        elif tlv_type == _TLV_RANGE_PROFILE and "range_profile" not in result:
            rp = _parse_range_profile(payload, body, tlv_len, num_range_bins)
            if rp is not None:
                result["range_profile"] = rp
                result["num_range_bins"] = len(rp)

        offset = body + tlv_len

    return result


def _parse_points(payload: bytes, offset: int, num: int) -> List[RadarPoint]:
    points: List[RadarPoint] = []
    pos = offset
    for _ in range(num):
        if pos + _POINT_SIZE_BYTES > len(payload):
            break
        x, y, z, vel = struct.unpack_from(_POINT_FMT, payload, pos)
        points.append(RadarPoint(x=float(x), y=float(y), z=float(z),
                                 velocity_mps=float(vel)))
        pos += _POINT_SIZE_BYTES
    return points


def _parse_range_doppler(payload: bytes, offset: int, tlv_len: int,
                         num_range: int, num_doppler: int) -> Optional[np.ndarray]:
    """Parse SDK 3 range-Doppler heatmap TLV (type 5).

    The payload is an unsigned, range-major matrix with dimensions defined by
    the active radar profile.  Return it as (range, Doppler), which maps
    directly to the ImageItem's vertical and horizontal axes respectively.
    """
    if not (0 < num_range <= 1024 and 0 < num_doppler <= 512):
        return None
    expected_bytes = num_doppler * num_range * 2  # uint16
    if tlv_len < expected_bytes or offset + expected_bytes > len(payload):
        return None

    try:
        raw = np.frombuffer(payload, dtype=np.uint16,
                            count=num_doppler * num_range,
                            offset=offset)
        rd = raw.reshape(num_range, num_doppler)
        return rd.copy()
    except Exception:
        return None


def _parse_range_profile(payload: bytes, offset: int, tlv_len: int,
                         num_range: int) -> Optional[np.ndarray]:
    """Parse SDK 3 range-profile TLV (type 2) as uint16 magnitudes."""
    if not (0 < num_range <= 1024):
        return None
    expected_bytes = num_range * 2
    if tlv_len < expected_bytes or offset + expected_bytes > len(payload):
        return None

    try:
        return np.frombuffer(payload, dtype=np.uint16,
                             count=num_range, offset=offset).copy()
    except Exception:
        return None


def _self_test():
    """Round-trip synthetically-built frame header + TLVs and assert parse."""
    import struct as _s

    num_obj = 2
    total = _HEADER_SIZE + 8 + num_obj * _POINT_SIZE_BYTES
    hdr = _s.pack(_HEADER_FMT, _MAGIC, 0x0303, total, 0xA1843, 7, 1000,
                  num_obj, 1, 0)

    pts = _s.pack("<" + "ffff" * num_obj,
                  1.0, 2.0, 0.5, -1.5,
                  -3.0, 4.0, 0.1, 3.2)
    pc_tlv = _s.pack("<II", _TLV_DETECTED_POINTS, len(pts)) + pts

    h = parse_frame_header(hdr)
    assert h is not None and h["num_detected_obj"] == 2 and h["total_packet_len"] == total

    res = parse_tlvs(pc_tlv, num_obj)
    assert len(res["points"]) == 2
    assert abs(res["points"][0].x - 1.0) < 1e-4
    assert abs(res["points"][0].velocity_mps + 1.5) < 1e-4
    assert abs(res["points"][1].x + 3.0) < 1e-4

    hdr_bad = _s.pack(_HEADER_FMT, b"\x02\x01\x04\x03\x06\x05\x08\x08",
                      0x0303, total, 0xA1843, 7, 1000, num_obj, 1, 0)
    assert parse_frame_header(hdr_bad) is None

    # Test SDK 3 range-Doppler TLV parse
    num_dop, num_rng = 4, 8
    rd_data = _s.pack("<" + "H" * (num_dop * num_rng),
                      *range(num_dop * num_rng))
    rd_tlv = _s.pack("<II", _TLV_RANGE_DOPPLER, len(rd_data)) + rd_data

    # The normal UART packet contains consecutive point-cloud and heatmap
    # TLVs.  Parsing both verifies that payload-only TLV lengths advance to
    # the next header correctly.
    res2 = parse_tlvs(pc_tlv + rd_tlv, num_obj, num_range_bins=num_rng,
                      num_doppler_bins=num_dop)
    assert "range_doppler" in res2
    assert len(res2["points"]) == num_obj
    assert res2["range_doppler"].shape == (num_rng, num_dop)
    assert res2["range_doppler"][3, 2] == 14
    assert res2["num_range_bins"] == num_rng
    assert res2["num_doppler_bins"] == num_dop

    print("radar.parser self-test OK")


if __name__ == "__main__":
    _self_test()
