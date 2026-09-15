"""NMEA 0183 sentence parser for GPS receivers."""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class GGA:
    """Global Positioning System Fix Data."""
    timestamp: float = 0.0
    latitude: float = 0.0
    lat_dir: str = "N"
    longitude: float = 0.0
    lon_dir: str = "E"
    fix_quality: int = 0
    satellites: int = 0
    hdop: float = 0.0
    altitude: float = 0.0
    sep_geoid: float = 0.0
    valid: bool = False

    @property
    def lat_decimal(self) -> float:
        deg = int(self.latitude / 100)
        minutes = self.latitude - deg * 100
        val = deg + minutes / 60.0
        return -val if self.lat_dir == "S" else val

    @property
    def lon_decimal(self) -> float:
        deg = int(self.longitude / 100)
        minutes = self.longitude - deg * 100
        val = deg + minutes / 60.0
        return -val if self.lon_dir == "W" else val

    @property
    def fix_type(self) -> str:
        types = {
            0: "No Fix", 1: "GPS Fix", 2: "DGPS Fix", 4: "RTK Fixed",
            5: "RTK Float", 6: "Estimated", 7: "Manual", 8: "Simulation",
        }
        return types.get(self.fix_quality, f"Unknown ({self.fix_quality})")

    @property
    def is_rtk_fixed(self) -> bool:
        return self.fix_quality == 4


@dataclass
class RMC:
    """Recommended Minimum Navigation Information."""
    timestamp: float = 0.0
    status: str = "V"
    latitude: float = 0.0
    lat_dir: str = "N"
    longitude: float = 0.0
    lon_dir: str = "E"
    speed_knots: float = 0.0
    course_deg: float = 0.0
    date: str = ""
    valid: bool = False

    @property
    def speed_mps(self) -> float:
        return self.speed_knots * 0.514444

    @property
    def course_rad(self) -> float:
        import math
        return math.radians(self.course_deg)


def _nmea_checksum(sentence: str) -> bool:
    if "*" not in sentence:
        return False
    body, checksum = sentence.split("*", 1)
    calc = 0
    for c in body.lstrip("$"):
        calc ^= ord(c)
    return calc == int(checksum, 16)


def _parse_fields(sentence: str) -> list[str]:
    if "*" in sentence:
        sentence = sentence.split("*")[0]
    parts = sentence.split(",")
    return [p.strip() for p in parts]


def parse_gga(sentence: str) -> GGA | None:
    if not _nmea_checksum(sentence):
        return None
    fields = _parse_fields(sentence)
    if len(fields) < 15 or not fields[0].endswith("GGA"):
        return None

    gga = GGA()
    try:
        gga.timestamp = float(fields[1]) if fields[1] else 0.0
        gga.latitude = float(fields[2]) if fields[2] else 0.0
        gga.lat_dir = fields[3] or "N"
        gga.longitude = float(fields[4]) if fields[4] else 0.0
        gga.lon_dir = fields[5] or "E"
        gga.fix_quality = int(fields[6]) if fields[6] else 0
        gga.satellites = int(fields[7]) if fields[7] else 0
        gga.hdop = float(fields[8]) if fields[8] else 0.0
        gga.altitude = float(fields[9]) if fields[9] else 0.0
        gga.sep_geoid = float(fields[11]) if fields[11] else 0.0
        gga.valid = True
    except (ValueError, IndexError):
        return None
    return gga


def parse_rmc(sentence: str) -> RMC | None:
    if not _nmea_checksum(sentence):
        return None
    fields = _parse_fields(sentence)
    if len(fields) < 12 or not fields[0].endswith("RMC"):
        return None

    rmc = RMC()
    try:
        rmc.timestamp = float(fields[1]) if fields[1] else 0.0
        rmc.status = fields[2] or "V"
        rmc.latitude = float(fields[3]) if fields[3] else 0.0
        rmc.lat_dir = fields[4] or "N"
        rmc.longitude = float(fields[5]) if fields[5] else 0.0
        rmc.lon_dir = fields[6] or "E"
        rmc.speed_knots = float(fields[7]) if fields[7] else 0.0
        rmc.course_deg = float(fields[8]) if fields[8] else 0.0
        rmc.date = fields[9] or ""
        rmc.valid = True
    except (ValueError, IndexError):
        return None
    return rmc


def parse_any(sentence: str) -> dict | None:
    """Parse any NMEA sentence, returning a dict with type and fields."""
    if not _nmea_checksum(sentence):
        return None
    fields = _parse_fields(sentence)
    if not fields:
        return None

    sentence_type = fields[0][-3:] if len(fields[0]) >= 3 else ""

    if sentence_type == "GGA":
        gga = parse_gga(sentence)
        if gga:
            return {"type": "GGA", "data": gga}
    elif sentence_type == "RMC":
        rmc = parse_rmc(sentence)
        if rmc:
            return {"type": "RMC", "data": rmc}

    return {"type": sentence_type, "data": fields}
